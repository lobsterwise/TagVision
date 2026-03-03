use std::{ops::Sub, sync::Arc};

use tokio::sync::{
	mpsc::{self, error::TrySendError},
	Mutex,
};

use crate::{
	cam::CameraFrame,
	config::{PoseEstimatorOption, RuntimeConfig, TagConfig, TagFilters},
	cv::{
		apriltag::{
			layout::AprilTagLayout, params::AprilTagDetectorParams, AprilTagDetections,
			AprilTagDetector,
		},
		distort::OpenCVCameraIntrinsics,
		geom::{PnPSolution, Pose3DWithError, PoseUpdate},
		solve::{p3p::P3P, pose_covariance, AprilTagHomographySolver, PnPSolver},
	},
	output::VisionOutput,
	util::Timer,
};

const QUEUE_SIZE: usize = 1;

/// A load balancer for multiple vision threads
pub struct VisionRuntime {
	sender: mpsc::Sender<VisionThreadInput>,
}

impl VisionRuntime {
	/// Creates a new VisionRuntime, also returning the receiver for vision outputs
	pub fn new(
		runtime_config: &RuntimeConfig,
		params: &AprilTagDetectorParams,
		tag_config: &TagConfig,
		layout: &AprilTagLayout,
		filters: &TagFilters,
		estimator: &PoseEstimatorOption,
	) -> (Self, mpsc::Receiver<VisionOutput>) {
		// Create the output channel
		let (output_sender, output_receiver) = mpsc::channel::<VisionOutput>(3);
		// Create the channel inputs to the vision threads
		let (sender, receiver) = mpsc::channel::<VisionThreadInput>(QUEUE_SIZE);

		let receiver = Arc::new(Mutex::new(receiver));
		let last_pose = Arc::new(Mutex::new(None));

		for _ in 0..runtime_config.vision_threads {
			let vision_thread_data = VisionThread::new(
				receiver.clone(),
				output_sender.clone(),
				estimator,
				last_pose.clone(),
				params.clone(),
				layout.clone(),
				tag_config.tag_size_meters(),
				filters.clone(),
			);

			tokio::spawn(vision_thread_data.run_forever());
		}

		let out = Self { sender };

		(out, output_receiver)
	}

	/// Sends a frame to the vision threads
	pub async fn send(
		&mut self,
		input: VisionThreadInput,
	) -> Result<(), mpsc::error::TrySendError<VisionThreadInput>> {
		if let e @ Err(TrySendError::Closed(..)) = self.sender.try_send(input) {
			e
		} else {
			Ok(())
		}
	}
}

/// Input to the vision thread
#[derive(Clone)]
pub struct VisionThreadInput {
	pub module: String,
	pub frame: CameraFrame,
	pub intrinsics: OpenCVCameraIntrinsics,
}

/// Data contained on the vision task
pub struct VisionThread {
	input: Arc<Mutex<mpsc::Receiver<VisionThreadInput>>>,
	output: mpsc::Sender<VisionOutput>,
	estimator: Box<dyn PnPSolver + Send>,
	last_pose: Arc<Mutex<Option<Pose3DWithError>>>,
	params: AprilTagDetectorParams,
	layout: AprilTagLayout,
	tag_size: f64,
	filters: TagFilters,
}

impl VisionThread {
	fn new(
		input_channel: Arc<Mutex<mpsc::Receiver<VisionThreadInput>>>,
		output_channel: mpsc::Sender<VisionOutput>,
		estimator: &PoseEstimatorOption,
		last_pose: Arc<Mutex<Option<Pose3DWithError>>>,
		params: AprilTagDetectorParams,
		layout: AprilTagLayout,
		tag_size: f64,
		filters: TagFilters,
	) -> Self {
		let estimator: Box<dyn PnPSolver + Send> = match estimator {
			PoseEstimatorOption::Homography => Box::new(AprilTagHomographySolver::new()),
			PoseEstimatorOption::P3P => Box::new(P3P::new()),
		};

		Self {
			input: input_channel,
			output: output_channel,
			estimator,
			last_pose,
			params,
			layout,
			tag_size,
			filters,
		}
	}

	async fn run_forever(mut self) {
		// We must initialize it here since it is not Send
		let detector = AprilTagDetector::new(self.params.clone());
		loop {
			self.run(&detector).await;
		}
	}

	async fn run(&mut self, detector: &AprilTagDetector) {
		let input = {
			let mut lock = self.input.lock().await;

			// It is OK to lock and await here as the tokio mutex is fair and will give to the next available task
			lock.recv().await
		};

		if let Some(input) = input {
			let mut timer = Timer::new();
			let detections = detector.detect_markers(&input.frame.image).await;
			let detection_time = timer.get_elapsed();

			timer.restart();
			let last_pose = self.last_pose.lock().await.clone();
			let pose = solve_tags(
				&mut self.estimator,
				&detections,
				&input.intrinsics,
				&self.layout,
				self.tag_size,
				&self.filters,
				last_pose,
			);
			let estimation_time = timer.get_elapsed();

			if let Some(pose) = &pose {
				*self.last_pose.lock().await = Some(pose.clone());
			}

			let update = pose.map(|pose| PoseUpdate {
				pose,
				timestamp: input.frame.timestamp,
			});

			let output = VisionOutput {
				module: input.module,
				update,
				frame: Some(input.frame.image),
				timestamp: input.frame.timestamp,
				detections: detections.to_vec(),
				detection_time: detection_time.as_secs_f32(),
				estimation_time: estimation_time.as_secs_f32(),
			};

			if let Err(e) = self.output.send(output).await {
				eprintln!("Output thread not available: {e}");
			}
		}
	}
}

/// Solve tags, returning the pose and average ambiguity
fn solve_tags(
	solver: &mut Box<dyn PnPSolver + Send>,
	detections: &AprilTagDetections,
	intrinsics: &OpenCVCameraIntrinsics,
	layout: &AprilTagLayout,
	tag_size: f64,
	filters: &TagFilters,
	last_pose: Option<Pose3DWithError>,
) -> Option<Pose3DWithError> {
	if detections.size() == 0 {
		return None;
	}

	let mut solved_poses = Vec::new();
	for detection in detections.iter() {
		if detection.is_filtered(filters) {
			continue;
		}

		let tag_corners_3d = layout.get_tag_corners(detection.id, tag_size);
		let Some(tag_corners_3d) = tag_corners_3d else {
			continue;
		};

		let solution = solver.solve(layout, &detection, intrinsics, tag_size);
		let Some(solution) = solution else {
			continue;
		};

		match solution {
			PnPSolution::Multi(poses) => {
				let best_pose = poses.into_iter().min_by(|x, y| {
					score_pose(x, last_pose.as_ref())
						.partial_cmp(&score_pose(y, last_pose.as_ref()))
						.unwrap_or(std::cmp::Ordering::Equal)
				});
				if let Some(pose) = best_pose {
					let covariance = pose_covariance(
						&pose.pose.r,
						&pose.pose.t,
						tag_corners_3d,
						intrinsics.fx,
						intrinsics.fy,
						0.5,
					);
					solved_poses.push(pose);
				}
			}
		}
	}

	if solved_poses.is_empty() {
		return None;
	}

	// Get the average pose

	let mut pose_sum = solved_poses
		.iter()
		.fold(Pose3DWithError::default(), |acc, pose| Pose3DWithError {
			error: acc.error + pose.error,
			pose: acc.pose.add(&pose.pose),
		});
	let n = solved_poses.len() as f64;
	pose_sum.pose.t /= n;
	pose_sum.pose.r = solved_poses.first().unwrap().pose.r;
	pose_sum.error /= n;

	Some(pose_sum)
}

/// Scores a given ambiguous pose option, with lower being better
fn score_pose(pose: &Pose3DWithError, last_pose: Option<&Pose3DWithError>) -> f64 {
	let dist_score = if let Some(last_pose) = last_pose {
		pose.pose.t.sub(last_pose.pose.t).norm()
	} else {
		0.0
	};

	// let err_score = pose.error;
	let err_score = 0.0;

	dist_score + err_score
}
