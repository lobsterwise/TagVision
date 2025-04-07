use crate::{
	cam::{Camera, CameraFrame, CameraSetupError, CaptureError},
	config::{ModuleConfig, RuntimeConfig},
};

/// An individual camera module
pub struct Module {
	config: ModuleConfig,
	camera: Camera,
}

impl Module {
	pub fn init(
		config: ModuleConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError> {
		let camera = Camera::from_config(config.camera.clone(), runtime_config)?;

		Ok(Self { config, camera })
	}

	pub fn get_frames(&mut self) -> Result<Vec<Result<CameraFrame, CaptureError>>, CaptureError> {
		self.camera.get_frames()
	}
}
