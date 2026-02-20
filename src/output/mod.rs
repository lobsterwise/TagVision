/// Utilities for interfacing with our nt4 crate
pub mod utils;

use std::{
	collections::{HashMap, HashSet},
	net::Ipv4Addr,
	str::FromStr,
	sync::Arc,
	time::Duration,
};

use image::GrayImage;
use nt_client::{publish::NewPublisherError, ClientHandle, NewClientOptions};
use tokio::sync::{mpsc::Receiver, Mutex};

use crate::{
	config::NetworkConfig,
	cv::{
		apriltag::AprilTagDetection,
		geom::PoseUpdate,
		img_utils::{fast_gray_to_rgb, ImageAllocator},
	},
	output::utils::{PubSub, ReconnectableClient},
	util::Timer,
};

static BASE_TABLE: &str = "TagVision";

/// Handles NetworkTables and CameraServer
pub struct Output {}

impl Output {
	pub async fn new(
		input: Receiver<VisionOutput>,
		network_config: NetworkConfig,
		modules: &HashSet<String>,
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
		let output_modules = Arc::new(Mutex::new(HashMap::new()));
		if !network_config.disabled {
			let fps_pub = fps_pub.clone();
			let output_modules = output_modules.clone();
			let modules = modules.clone();

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
					tokio::spawn(async move {
						// I want to initialize all the topics asynchronously, but it creates data type mismatch errors, so we have to do it synchronously
						let mut output_modules2: HashMap<String, OutputModule> = HashMap::new();
						for module_id in modules {
							let module =
								OutputModule::new(&module_id, &client, &reconn_client).await;
							let module =
								match module {
									Ok(module) => module,
									Err(e) => {
										eprintln!("Failed to initialize output for module '{module_id}': {e}");
										continue;
									}
								};

							output_modules2.insert(module_id.clone(), module);
						}
						*output_modules.lock().await = output_modules2;

						if let Ok(pubsub) = reconn_client
							.get_topic(
								format!("{BASE_TABLE}/FPS"),
								Duration::from_millis(10),
								&client,
							)
							.await
						{
							*fps_pub.lock().await = Some(pubsub);
						}

						println!("Output setup complete");
					});
				},
			);
		}

		let thread_data = OutputThread {
			input,
			fps_timer: Timer::new(),
			event_count: 0,
			detection_count: 0,
			image_allocator: ImageAllocator::new(),
			output_modules,
			fps_pub,
			detection_time_sum: 0.0,
			estimation_time_sum: 0.0,
		};

		tokio::spawn(thread_data.run_forever());

		Self {}
	}
}

/// Data contained on the output task
struct OutputThread {
	image_allocator: ImageAllocator,
	// Input
	input: Receiver<VisionOutput>,
	// Outputs
	output_modules: Arc<Mutex<HashMap<String, OutputModule>>>,
	fps_pub: Arc<Mutex<Option<PubSub<f64>>>>,
	// Stats
	fps_timer: Timer,
	event_count: u16,
	detection_count: u16,
	detection_time_sum: f32,
	estimation_time_sum: f32,
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
				// let mut rgb_image = DynamicImage::ImageLuma8(input.frame.image).into_rgb8();
				for detection in output.detections {
					detection.draw(rgb_image);
				}
			}

			if let Some(update) = output.update {
				if let Ok(mut lock) = self.output_modules.try_lock() {
					if let Some(module) = lock.get_mut(&output.module) {
						self.detection_count += 1;
						let _ = module.output(update).await;
					}
				}
			}

			self.event_count += 1;
			self.detection_time_sum += output.detection_time;
			self.estimation_time_sum += output.estimation_time;
		}

		// Calculate stats
		let stats_interval = 1.5;
		if self
			.fps_timer
			.interval(Duration::from_secs_f32(stats_interval))
		{
			let event_count_f32 = self.event_count as f32;
			let fps = event_count_f32 / stats_interval;
			let detection_ratio = if self.event_count == 0 {
				0.0
			} else {
				self.detection_count as f32 / event_count_f32
			};

			let (detection_time, estimation_time) = if self.event_count == 0 {
				(0.0, 0.0)
			} else {
				(
					self.detection_time_sum / event_count_f32,
					self.estimation_time_sum / event_count_f32,
				)
			};

			self.event_count = 0;
			self.detection_count = 0;
			self.detection_time_sum = 0.0;
			self.estimation_time_sum = 0.0;

			if let Ok(Some(lock)) = self.fps_pub.try_lock().as_deref_mut() {
				lock.publish(fps as f64).await;
			}
			println!(
				"FPS: {fps:.3}; Detection %: {:.2}; Detection Time: {:.1}ms; Estimation Time: {:.1}ms",
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
	pub frame: Option<Arc<GrayImage>>,
	pub detections: Vec<AprilTagDetection>,
	pub detection_time: f32,
	pub estimation_time: f32,
}

/// NT pub/sub for a module
struct OutputModule {
	data_pubsub: PubSub<Vec<f64>>,
	data_buffer: Vec<f64>,
}

impl OutputModule {
	async fn new(
		module_id: &str,
		client: &ClientHandle,
		reconnectable_client: &ReconnectableClient,
	) -> Result<Self, NewPublisherError> {
		let table = format!("{BASE_TABLE}/{module_id}");

		let data_pubsub = reconnectable_client
			.get_topic(format!("{table}/Data"), Duration::from_micros(500), client)
			.await?;

		Ok(Self {
			data_pubsub,
			data_buffer: vec![0.0; 7],
		})
	}

	/// Outputs an update to this module
	async fn output(&mut self, update: PoseUpdate) {
		let data = update.as_array();
		self.data_buffer.copy_from_slice(&data);

		self.data_pubsub.publish(self.data_buffer.clone()).await;
	}
}
