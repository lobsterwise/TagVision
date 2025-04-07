use image::DynamicImage;
use tokio::sync::mpsc::{error::SendError, Receiver, Sender};

use crate::{
	cam::CameraFrame,
	config::RuntimeConfig,
	cv::{
		apriltag::{params::AprilTagDetectorParams, AprilTagDetector},
		geom::{PnPSolution, Pose3DWithError, PoseUpdate},
		img_utils::ImageAllocator,
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
	) -> (Self, Receiver<VisionOutput>) {
		// Create the output channel
		let (output_sender, output_receiver) = tokio::sync::mpsc::channel::<VisionOutput>(1);
		// Create the channel inputs to the vision threads
		let mut senders = Vec::with_capacity(2);
		for _ in 0..2 {
			let (sender, receiver) = tokio::sync::mpsc::channel::<VisionThreadInput>(1);

			let vision_thread_data = VisionThread::new(
				receiver,
				output_sender.clone(),
				params.clone(),
				runtime_config.clone(),
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
}

/// Data contained on the vision task
pub struct VisionThread {
	input: Receiver<VisionThreadInput>,
	output: Sender<VisionOutput>,
	params: AprilTagDetectorParams,
	runtime_config: RuntimeConfig,
	image_allocator: ImageAllocator,
}

impl VisionThread {
	fn new(
		input_channel: Receiver<VisionThreadInput>,
		output_channel: Sender<VisionOutput>,
		params: AprilTagDetectorParams,
		runtime_config: RuntimeConfig,
	) -> Self {
		Self {
			input: input_channel,
			output: output_channel,
			params,
			runtime_config,
			image_allocator: ImageAllocator::new(),
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
			let pose = PoseUpdate {
				pose: PnPSolution::Single(Pose3DWithError::default()),
				timestamp: input.frame.timestamp,
			};

			// Convert to RGB to draw the resulting tags
			// let rgb_image = self
			// 	.image_allocator
			// 	.get_rgb_image(input.frame.image.width(), input.frame.image.height());
			// fast_gray_to_rgb(&input.frame.image, rgb_image);
			let mut rgb_image = DynamicImage::ImageLuma8(input.frame.image).into_rgb8();
			detections.draw(&mut rgb_image);

			let output = VisionOutput {
				module: input.module,
				update: pose,
				frame: Some(rgb_image),
			};

			if let Err(e) = self.output.send(output).await {
				eprintln!("Output thread not available: {e}");
			}
		}
	}
}
