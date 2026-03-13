/// CameraServer protocol
mod cs;
/// Frame logging
mod photo_log;
/// Utilities for interfacing with our nt4 crate
pub mod utils;

use std::{
	collections::{HashMap, HashSet},
	net::Ipv4Addr,
	str::FromStr,
	sync::Arc,
	time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use image::GrayImage;
use nalgebra::UnitQuaternion;
use nt_client::{
	math::{Pose3d, Quaternion, Rotation3d, Translation3d},
	r#struct::{Struct, StructData},
	ClientHandle, NewClientOptions,
};
use nt_client_macros::StructData;
use tokio::sync::{
	mpsc::{Receiver, Sender},
	Mutex,
};
use tracing::{error, info};

use crate::{
	config::{NetworkConfig, PhotoLoggingConfig},
	cv::{
		apriltag::{layout::AprilTagLayout, AprilTagDetection},
		geom::{Pose3D, PoseUpdate},
		img_utils::{fast_gray_to_rgb, ImageAllocator},
	},
	output::{
		cs::{CameraServer, CameraServerInput},
		photo_log::PhotoLogger,
		utils::{PubSub, ReconnectableClient, StructArray, StructDataSize},
	},
	util::{MovingAverage, Timer},
};

static BASE_TABLE: &str = "TagVision";

/// Handles NetworkTables and CameraServer
pub struct Output {}

impl Output {
	pub async fn new(
		input: Receiver<VisionOutput>,
		network_config: NetworkConfig,
		modules: &HashSet<String>,
		layout: AprilTagLayout,
		photo_logging_config: PhotoLoggingConfig,
	) -> Self {
		let addr = if let Some(address) = &network_config.address {
			nt_client::NTAddr::Custom(Ipv4Addr::from_str(address).unwrap())
		} else if let Some(team_number) = network_config.team_number {
			nt_client::NTAddr::TeamNumber(team_number)
		} else {
			panic!("No address provided")
		};

		let reconnect_interval = Duration::from_secs_f32(network_config.reconnect_interval);

		let fps_pub = Arc::new(Mutex::new(None));
		let schema_pubs = Arc::new(Mutex::new(None));
		let output_modules = Arc::new(Mutex::new(HashMap::new()));
		let camera_server = Arc::new(Mutex::new(None));
		if !network_config.disabled {
			let modules: Vec<_> = modules.iter().cloned().collect();
			let fps_pub = fps_pub.clone();
			let schema_pubs = schema_pubs.clone();
			let output_modules = output_modules.clone();
			let camera_server = camera_server.clone();

			let options = NewClientOptions {
				addr,
				name: network_config.name.clone(),
				secure_port: None,
				..Default::default()
			};
			ReconnectableClient::spawn(
				options,
				reconnect_interval,
				move |client, reconn_client| {
					let client = client.handle().clone();
					let reconn_client = reconn_client.clone();

					let modules = modules.clone();
					let fps_pub = fps_pub.clone();
					let schema_pubs = schema_pubs.clone();
					let output_modules = output_modules.clone();
					let camera_server = camera_server.clone();
					tokio::spawn(async move {
						// I want to initialize all the topics asynchronously, but it creates data type mismatch errors, so we have to do it synchronously
						let mut output_modules2: HashMap<String, OutputModule> = HashMap::new();
						for module_id in &modules {
							let module =
								OutputModule::new(module_id, &client, &reconn_client).await;

							output_modules2.insert(module_id.clone(), module);
						}
						*output_modules.lock().await = output_modules2;

						info!("Module output publishers started");

						let pubsub = reconn_client
							.get_topic(
								format!("{BASE_TABLE}/FPS"),
								Duration::from_millis(10),
								&client,
							)
							.await;
						*fps_pub.lock().await = Some(pubsub);

						info!("FPS publisher started");

						let pubs = SchemaPublishers::new(
							&client,
							&reconn_client,
							network_config.enable_wpi_schemas,
						)
						.await;
						*schema_pubs.lock().await = Some(pubs);

						info!("Schema publishers started");

						if network_config.camera_server {
							let (cs_tx, cs_rx) = tokio::sync::mpsc::channel(5);
							match CameraServer::new(&modules, &client, &reconn_client, cs_rx).await
							{
								Ok(cs) => {
									tokio::spawn(async move {
										cs.run_forever().await;
									});
									*camera_server.lock().await = Some(cs_tx);
								}
								Err(e) => {
									error!("Failed to initialize camera server: {e}");
								}
							}
						}

						info!("Output setup complete");
					});
				},
			);
		}

		let thread_data = OutputThread {
			input,
			layout,
			stats_timer: Timer::new(),
			last_frame_time: None,
			fps_average: MovingAverage::new(100),
			event_count: 0,
			detection_count: 0,
			image_allocator: ImageAllocator::new(),
			output_modules,
			fps_pub,
			schema_pubs,
			camera_server,
			photo_logger: PhotoLogger::new(photo_logging_config),
			detection_time_sum: 0.0,
			estimation_time_sum: 0.0,
			full_pipeline_sum: 0.0,
		};

		tokio::spawn(thread_data.run_forever());

		Self {}
	}
}

/// Data contained on the output task
struct OutputThread {
	image_allocator: ImageAllocator,
	layout: AprilTagLayout,
	// Input
	input: Receiver<VisionOutput>,
	// Outputs
	output_modules: Arc<Mutex<HashMap<String, OutputModule>>>,
	fps_pub: Arc<Mutex<Option<PubSub<f64>>>>,
	schema_pubs: Arc<Mutex<Option<SchemaPublishers>>>,
	camera_server: Arc<Mutex<Option<Sender<CameraServerInput>>>>,
	photo_logger: PhotoLogger,
	// Stats
	stats_timer: Timer,
	last_frame_time: Option<Instant>,
	fps_average: MovingAverage,
	event_count: u16,
	detection_count: u16,
	detection_time_sum: f32,
	estimation_time_sum: f32,
	full_pipeline_sum: f32,
}

impl OutputThread {
	async fn run_forever(mut self) {
		loop {
			self.run().await;
		}
	}

	async fn run(&mut self) {
		if let Some(output) = self.input.recv().await {
			if let Some(frame) = output.frame {
				let rgb_image = self
					.image_allocator
					.get_rgb_image(frame.width(), frame.height());
				fast_gray_to_rgb(&frame, rgb_image);
				for detection in &output.detections {
					detection.draw(rgb_image);
				}
				if let Ok(Some(lock)) = self.camera_server.try_lock().as_deref() {
					let _ = lock.try_send(CameraServerInput {
						module_id: output.module.clone(),
						frame: rgb_image.clone(),
					});
				}

				let _ = self.photo_logger.log(&output.module, rgb_image);
				self.detection_count += 1;
			}

			if let Some(update) = output.update {
				self.detection_count += 1;

				if let Ok(mut lock) = self.output_modules.try_lock() {
					if let Some(module) = lock.get_mut(&output.module) {
						let _ = module
							.output(update, &output.detections, &self.layout)
							.await;
					}
				}
			}

			self.detection_time_sum += output.detection_time;
			self.estimation_time_sum += output.estimation_time;
			let now = SystemTime::now()
				.duration_since(UNIX_EPOCH)
				.unwrap_or_default()
				.as_secs_f64();
			let diff = now - output.timestamp;
			self.full_pipeline_sum += diff as f32;
		}

		if let Ok(Some(lock)) = self.schema_pubs.try_lock().as_deref_mut() {
			lock.publish().await;
		}

		// Calculate stats

		let now = Instant::now();
		let fps = if let Some(last_frame_time) = &self.last_frame_time {
			let secs = (now - *last_frame_time).as_secs_f64();
			if secs == 0.0 {
				0.0
			} else {
				1.0 / secs
			}
		} else {
			0.0
		};
		let fps = self.fps_average.calculate(fps);
		self.last_frame_time = Some(now);

		if let Ok(Some(lock)) = self.fps_pub.try_lock().as_deref_mut() {
			lock.publish(fps as f64);
		}

		// Console printing
		let stats_interval = 1.5;
		if self
			.stats_timer
			.interval(Duration::from_secs_f32(stats_interval))
		{
			let n = if self.event_count == 0 {
				1.0
			} else {
				self.event_count as f32
			};

			let detection_ratio = self.detection_count as f32 / n;
			let detection_time = self.detection_time_sum as f32 / n;
			let estimation_time = self.estimation_time_sum as f32 / n;
			let full_pipeline_time = self.full_pipeline_sum as f32 / n;

			self.event_count = 0;
			self.detection_count = 0;
			self.detection_time_sum = 0.0;
			self.estimation_time_sum = 0.0;
			self.full_pipeline_sum = 0.0;

			info!(
				"FPS: {fps:.1}; Detection %: {:.2}; Detection Time: {:.1}ms; Estimation Time: {:.1}ms; Full Pipeline: {full_pipeline_time:.1}ms",
				detection_ratio * 100.0,
				detection_time * 1000.0,
				estimation_time * 1000.0,
			);
		}
	}
}

/// Output from the vision thread and input to the output thread
pub struct VisionOutput {
	pub module: String,
	pub update: Option<PoseUpdate>,
	pub timestamp: f64,
	pub frame: Option<Arc<GrayImage>>,
	pub detections: Vec<AprilTagDetection>,
	pub detection_time: f32,
	pub estimation_time: f32,
}

/// NT pub/sub for a module
struct OutputModule {
	data_pubsub: PubSub<Struct<TagVisionPoseUpdate>>,
	tags_pubsub: PubSub<StructArray<TagVisionDetection>>,
}

impl OutputModule {
	async fn new(
		module_id: &str,
		client: &ClientHandle,
		reconnectable_client: &ReconnectableClient,
	) -> Self {
		let table = format!("{BASE_TABLE}/{module_id}");

		let data_pubsub = reconnectable_client
			.get_topic(format!("{table}/Data"), Duration::from_micros(500), client)
			.await;
		let tags_pubsub = reconnectable_client
			.get_topic(format!("{table}/Tags"), Duration::from_millis(10), client)
			.await;

		Self {
			data_pubsub,
			tags_pubsub,
		}
	}

	/// Outputs an update to this module
	async fn output(
		&mut self,
		update: PoseUpdate,
		detections: &[AprilTagDetection],
		layout: &AprilTagLayout,
	) {
		self.data_pubsub
			.publish(Struct(TagVisionPoseUpdate::from_update(update.clone())));

		// let default = Pose3D::default();
		// let tags = detections
		// 	.iter()
		// 	.map(|x| {
		// 		let pose = layout.get_tag_pose(x.id).unwrap_or(&default);
		// 		let pose = Pose3D::from_isometry(
		// 			pose.to_isometry() * Isometry3::translation(0.0, 0.0, 0.0),
		// 		);
		// 		let pose = create_sendable_pose(&pose);

		// 		let corners = layout.get_tag_corners(x.id).unwrap_or(Matrix3x4::zeros());
		// 		let c1 = Translation3d {
		// 			x: corners.m11,
		// 			y: corners.m21,
		// 			z: corners.m31,
		// 		};
		// 		let c2 = Translation3d {
		// 			x: corners.m12,
		// 			y: corners.m22,
		// 			z: corners.m32,
		// 		};
		// 		let c3 = Translation3d {
		// 			x: corners.m13,
		// 			y: corners.m23,
		// 			z: corners.m33,
		// 		};
		// 		let c4 = Translation3d {
		// 			x: corners.m14,
		// 			y: corners.m24,
		// 			z: corners.m34,
		// 		};

		// 		TagVisionDetection {
		// 			id: x.id as i32,
		// 			pose,
		// 			c1,
		// 			c2,
		// 			c3,
		// 			c4,
		// 		}
		// 	})
		// 	.collect();

		// self.tags_pubsub.publish(StructArray(tags));
	}
}

/// A PoseUpdate that is sent over NT
#[derive(Clone, Debug, StructData)]
#[structdata(
	schema = "Pose3d pose; double error; double timestamp; int16 tag; double covx; double covy; double covz; double covrx; double covry; double covrz;"
)]
struct TagVisionPoseUpdate {
	#[structdata(nested)]
	pose: Pose3d,
	error: f64,
	timestamp: f64,
	tag: i16,
	covx: f64,
	covy: f64,
	covz: f64,
	covrx: f64,
	covry: f64,
	covrz: f64,
}

impl TagVisionPoseUpdate {
	fn from_update(update: PoseUpdate) -> Self {
		Self {
			pose: create_sendable_pose(&update.pose.pose),
			error: update.pose.error,
			timestamp: update.timestamp,
			tag: update.tag as i16,
			covx: update.covariance.x,
			covy: update.covariance.y,
			covz: update.covariance.z,
			covrx: update.covariance.rx,
			covry: update.covariance.ry,
			covrz: update.covariance.rz,
		}
	}
}

/// A tag detection that is sent over NT
#[derive(Clone, Debug, StructData)]
#[structdata(
	schema = "Pose3d pose; int32 id; Translation3d c1; Translation3d c2; Translation3d c3; Translation3d c4;"
)]
struct TagVisionDetection {
	#[structdata(nested)]
	pose: Pose3d,
	id: i32,
	#[structdata(nested)]
	c1: Translation3d,
	#[structdata(nested)]
	c2: Translation3d,
	#[structdata(nested)]
	c3: Translation3d,
	#[structdata(nested)]
	c4: Translation3d,
}

impl StructDataSize for TagVisionDetection {
	fn size() -> usize {
		156
	}
}

/// Publishers for schema data
struct SchemaPublishers {
	tag_vision_pose_update: PubSub<rmpv::Value>,
	tag_vision_detection: PubSub<rmpv::Value>,
	quaternion: Option<PubSub<rmpv::Value>>,
	rotation3d: Option<PubSub<rmpv::Value>>,
	translation3d: Option<PubSub<rmpv::Value>>,
	pose3d: Option<PubSub<rmpv::Value>>,
}

impl SchemaPublishers {
	pub async fn new(
		client: &ClientHandle,
		reconnectable_client: &ReconnectableClient,
		enable_wpi: bool,
	) -> Self {
		let tag_vision_pose_update = reconnectable_client
			.get_topic(
				format!("/.schema/struct:TagVisionPoseUpdate"),
				Duration::from_millis(100),
				&client,
			)
			.await;

		let tag_vision_detection = reconnectable_client
			.get_topic(
				format!("/.schema/struct:TagVisionDetection"),
				Duration::from_millis(100),
				&client,
			)
			.await;

		let quaternion = if enable_wpi {
			Some(
				reconnectable_client
					.get_topic(
						format!("/.schema/struct:Quaternion"),
						Duration::from_millis(100),
						&client,
					)
					.await,
			)
		} else {
			None
		};

		let rotation3d = if enable_wpi {
			Some(
				reconnectable_client
					.get_topic(
						format!("/.schema/struct:Rotation3d"),
						Duration::from_millis(100),
						&client,
					)
					.await,
			)
		} else {
			None
		};

		let translation3d = if enable_wpi {
			Some(
				reconnectable_client
					.get_topic(
						format!("/.schema/struct:Translation3d"),
						Duration::from_millis(100),
						&client,
					)
					.await,
			)
		} else {
			None
		};

		let pose3d = if enable_wpi {
			Some(
				reconnectable_client
					.get_topic(
						format!("/.schema/struct:Pose3d"),
						Duration::from_millis(100),
						&client,
					)
					.await,
			)
		} else {
			None
		};

		Self {
			tag_vision_pose_update,
			tag_vision_detection,
			quaternion,
			rotation3d,
			translation3d,
			pose3d,
		}
	}

	pub async fn publish(&mut self) {
		self.tag_vision_pose_update.publish(rmpv::Value::Binary(
			TagVisionPoseUpdate::schema().0.into_bytes(),
		));

		self.tag_vision_detection.publish(rmpv::Value::Binary(
			TagVisionDetection::schema().0.into_bytes(),
		));

		if let Some(pubsub) = &mut self.quaternion {
			pubsub.publish(rmpv::Value::Binary(Quaternion::schema().0.into_bytes()));
		}

		if let Some(pubsub) = &mut self.rotation3d {
			pubsub.publish(rmpv::Value::Binary(Rotation3d::schema().0.into_bytes()));
		}

		if let Some(pubsub) = &mut self.translation3d {
			let _ = pubsub.publish(rmpv::Value::Binary(Translation3d::schema().0.into_bytes()));
		}

		if let Some(pubsub) = &mut self.pose3d {
			pubsub.publish(rmpv::Value::Binary(Pose3d::schema().0.into_bytes()));
		}
	}
}

fn create_sendable_pose(pose: &Pose3D) -> Pose3d {
	let quat = UnitQuaternion::from_matrix(&pose.r);

	Pose3d {
		translation: Translation3d {
			x: pose.t.x,
			y: pose.t.y,
			z: pose.t.z,
		},
		rotation: Rotation3d {
			quaternion: Quaternion {
				x: quat.i,
				y: quat.j,
				z: quat.k,
				w: quat.w,
			},
		},
	}
}
