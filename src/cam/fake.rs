use std::{
	sync::Arc,
	time::{Duration, SystemTime, UNIX_EPOCH},
};

use image::GrayImage;

use crate::{
	config::{CameraConfig, RuntimeConfig},
	util::Timer,
};

use super::{CameraBackend, CameraFrame, CameraSetupError, CaptureError};

pub struct FakeCamera {
	image: Arc<GrayImage>,
	/// Timer to produce images at a constant FPS
	timer: Timer,
	interval: f32,
}

impl CameraBackend for FakeCamera {
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError> {
		let _ = runtime_config;

		let image = image::open(&config.device_id)
			.map_err(|e| CameraSetupError::CameraNotFound(e.to_string()))?;
		let image = Arc::new(image.into());

		Ok(Self {
			timer: Timer::new(),
			image,
			interval: 1.0 / config.fps as f32,
		})
	}

	fn get_frames(
		&mut self,
		buf: &mut Vec<Result<CameraFrame, CaptureError>>,
	) -> Result<(), CaptureError> {
		if self.timer.interval(Duration::from_secs_f32(self.interval)) {
			let timestamp = SystemTime::now()
				.duration_since(UNIX_EPOCH)
				.unwrap_or_default()
				.as_millis();

			buf.push(Ok(CameraFrame {
				timestamp,
				image: self.image.clone(),
			}));
		}

		Ok(())
	}
}
