use tokio::sync::mpsc::{error::SendError, Receiver, Sender};

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
};

/// A load balancer for multiple vision threads
pub struct VisionRuntime {
	senders: Vec<Sender<VisionThreadInput>>,
	index: usize,
}

impl VisionRuntime {
	/// Creates a new VisionRuntime, also returning the receiver for vision outputs
	pub fn new(
		params: &AprilTagDetectorParams,
		runtime_config: &RuntimeConfig,
		tag_config: &TagConfig,
		layout: &AprilTagLayout,
	) -> (Self, Receiver<VisionOutput>) {
		// Create the output channel
		let (output_sender, output_receiver) = tokio::sync::mpsc::channel::<VisionOutput>(1);
		// Create the channel inputs to the vision threads
		let runtime_count = 2;
		let mut senders = Vec::with_capacity(runtime_count);
		for _ in 0..runtime_count {
			let (sender, receiver) = tokio::sync::mpsc::channel::<VisionThreadInput>(1);

			let vision_thread_data = VisionThread::new(
				receiver,
				output_sender.clone(),
				params.clone(),
				runtime_config.clone(),
				layout.clone(),
				tag_config.tag_size,
			);

			tokio::spawn(vision_thread_data.run_forever());

			senders.push(sender);
		}

		let out = Self { senders, index: 0 };

		(out, output_receiver)
	}

	pub async fn send(
		&mut self,
		input: VisionThreadInput,
	) -> Result<(), SendError<VisionThreadInput>> {
		self.index += 1;
		self.index %= self.senders.len();

		if self.senders.len() != 0 {
			self.senders[self.index].send(input).await
		} else {
			Ok(())
		}
	}
}

/// Input to the vision thread
pub struct VisionThreadInput {
	pub module: String,
	pub frame: CameraFrame,
	pub intrinsics: OpenCVCameraIntrinsics,
}

/// Data contained on the vision task
pub struct VisionThread {
	input: Receiver<VisionThreadInput>,
	output: Sender<VisionOutput>,
	params: AprilTagDetectorParams,
	runtime_config: RuntimeConfig,
	layout: AprilTagLayout,
	tag_size: f64,
}

impl VisionThread {
	fn new(
		input_channel: Receiver<VisionThreadInput>,
		output_channel: Sender<VisionOutput>,
		params: AprilTagDetectorParams,
		runtime_config: RuntimeConfig,
		layout: AprilTagLayout,
		tag_size: f64,
	) -> Self {
		Self {
			input: input_channel,
			output: output_channel,
			params,
			runtime_config,
			layout,
			tag_size,
		}
	}

	async fn run_forever(mut self) {
		let detector = AprilTagDetector::new(self.params.clone());
		loop {
			self.run(&detector).await;
			// tokio::time::sleep(Duration::from_secs_f32(
			// 	self.runtime_config
			// 		.update_rate
			// 		.unwrap_or(DEFAULT_UPDATE_RATE)
			// 		/ 1000.0,
			// ))
			// .await;
		}
	}

	async fn run(&mut self, detector: &AprilTagDetector) {
		if let Some(input) = self.input.recv().await {
			let detections = detector.detect_markers(&input.frame.image);
			// dbg!(&detections);

			let pose = solve_tags(&detections, &input.intrinsics, &self.layout, self.tag_size);
			dbg!(&pose);
			if let Some(pose) = pose {
				let update = PoseUpdate {
					pose,
					timestamp: input.frame.timestamp,
				};

				let output = VisionOutput {
					module: input.module,
					update,
					frame: Some(input.frame.image),
					detections: detections.to_vec(),
				};

				if let Err(e) = self.output.send(output).await {
					eprintln!("Output thread not available: {e}");
				}
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
	let mut solver = P3P::new(intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
	let mut solved_poses = Vec::new();
	for detection in detections.iter() {
		let tag_corners_3d = layout.get_tag_corners(detection.id, tag_size);
		let Some(tag_corners_3d) = tag_corners_3d else {
			continue;
		};

		dbg!(&detection.corners);
		// dbg!(&intrinsics);
		let undistorted = detection.get_undistorted_corners(intrinsics);
		dbg!(&undistorted);
		let solution = solver.solve(tag_corners_3d, undistorted);
		let Some(solution) = solution else {
			continue;
		};

		match solution {
			PnPSolution::Single(pose) => {
				solved_poses.push(pose.pose);
			}
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
