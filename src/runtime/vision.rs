use std::sync::Arc;

use tokio::sync::{
	mpsc::{self, error::TrySendError},
	Mutex,
};

use crate::{
	cam::CameraFrame,
	config::{RuntimeConfig, TagConfig},
	cv::{
		apriltag::{
			layout::AprilTagLayout, params::AprilTagDetectorParams, AprilTagDetections,
			AprilTagDetector,
		},
		distort::OpenCVCameraIntrinsics,
		geom::{PnPSolution, Pose3D, PoseUpdate},
		solve::{p3p::P3P, PnPSolver},
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
	) -> (Self, mpsc::Receiver<VisionOutput>) {
		// Create the output channel
		let (output_sender, output_receiver) = mpsc::channel::<VisionOutput>(3);
		// Create the channel inputs to the vision threads
		let (sender, receiver) = mpsc::channel::<VisionThreadInput>(QUEUE_SIZE);

		let receiver = Arc::new(Mutex::new(receiver));

		for _ in 0..runtime_config.vision_threads {
			let vision_thread_data = VisionThread::new(
				receiver.clone(),
				output_sender.clone(),
				params.clone(),
				layout.clone(),
				tag_config.tag_size,
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
	params: AprilTagDetectorParams,
	layout: AprilTagLayout,
	tag_size: f64,
}

impl VisionThread {
	fn new(
		input_channel: Arc<Mutex<mpsc::Receiver<VisionThreadInput>>>,
		output_channel: mpsc::Sender<VisionOutput>,
		params: AprilTagDetectorParams,
		layout: AprilTagLayout,
		tag_size: f64,
	) -> Self {
		Self {
			input: input_channel,
			output: output_channel,
			params,
			layout,
			tag_size,
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
			let pose = solve_tags(&detections, &input.intrinsics, &self.layout, self.tag_size);
			let estimation_time = timer.get_elapsed();

			let update = pose.map(|pose| PoseUpdate {
				pose,
				timestamp: input.frame.timestamp,
			});

			let output = VisionOutput {
				module: input.module,
				update,
				frame: Some(input.frame.image),
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

fn solve_tags(
	detections: &AprilTagDetections,
	intrinsics: &OpenCVCameraIntrinsics,
	layout: &AprilTagLayout,
	tag_size: f64,
) -> Option<Pose3D> {
	if detections.size() == 0 {
		return None;
	}

	let mut solver = P3P::new(intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
	let mut solved_poses = Vec::new();
	for detection in detections.iter() {
		let tag_corners_3d = layout.get_tag_corners(detection.id, tag_size);
		let Some(tag_corners_3d) = tag_corners_3d else {
			continue;
		};

		let undistorted = detection.get_undistorted_corners(intrinsics);
		let solution = solver.solve(tag_corners_3d, undistorted);
		let Some(solution) = solution else {
			continue;
		};

		match solution {
			PnPSolution::Multi(poses) => {
				let pose_with_min_err = poses.into_iter().min_by(|x, y| {
					x.error
						.partial_cmp(&y.error)
						.unwrap_or(std::cmp::Ordering::Equal)
				});
				if let Some(pose) = pose_with_min_err {
					solved_poses.push(pose.pose);
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
		.fold(Pose3D::default(), |acc, pose| acc.add(pose));
	let n = solved_poses.len() as f64;
	pose_sum.x /= n;
	pose_sum.y /= n;
	pose_sum.z /= n;
	pose_sum.rx /= n;
	pose_sum.ry /= n;
	pose_sum.rz /= n;

	Some(pose_sum)
}
