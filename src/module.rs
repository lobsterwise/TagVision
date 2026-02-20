use crate::{
	cam::{Camera, CameraFrame, CameraSetupError, CaptureError},
	config::{ModuleConfig, RuntimeConfig},
	cv::distort::OpenCVCameraIntrinsics,
};

/// An individual camera module
pub struct Module {
	camera: Camera,
	intrinsics: OpenCVCameraIntrinsics,
}

impl Module {
	/// Initializes the module and camera
	pub fn init(
		config: ModuleConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError> {
		let camera = Camera::from_config(config.camera.clone(), runtime_config, config.max_errors)?;
		let intrinsics = OpenCVCameraIntrinsics::from_calib(&config.camera.intrinsics);

		Ok(Self { camera, intrinsics })
	}

	/// Gets frames from the module's camera
	pub fn get_frames(&mut self) -> Result<&Vec<Result<CameraFrame, CaptureError>>, CaptureError> {
		self.camera.get_frames()
	}

	/// Gets intrinsics for the module
	pub fn get_intrinsics(&self) -> &OpenCVCameraIntrinsics {
		&self.intrinsics
	}

	/// Self-checks and returns whether the module needs a restart
	pub fn self_check(&mut self) -> bool {
		self.camera.self_check()
	}
}
