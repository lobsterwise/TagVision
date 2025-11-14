use crate::{
	cam::{Camera, CameraFrame, CameraSetupError, CaptureError},
	config::{ModuleConfig, RuntimeConfig},
	cv::distort::OpenCVCameraIntrinsics,
};

/// An individual camera module
pub struct Module {
	config: ModuleConfig,
	camera: Camera,
	intrinsics: OpenCVCameraIntrinsics,
}

impl Module {
	pub fn init(
		config: ModuleConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError> {
		let camera = Camera::from_config(config.camera.clone(), runtime_config)?;
		let intrinsics = OpenCVCameraIntrinsics::from_calib(&config.camera.intrinsics);

		Ok(Self {
			config,
			camera,
			intrinsics,
		})
	}

	pub fn get_frames(&mut self) -> Result<Vec<Result<CameraFrame, CaptureError>>, CaptureError> {
		self.camera.get_frames()
	}

	pub fn get_intrinsics(&self) -> &OpenCVCameraIntrinsics {
		&self.intrinsics
	}
}
