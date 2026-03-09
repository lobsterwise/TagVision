use std::{
	sync::Arc,
	time::{Duration, SystemTime, UNIX_EPOCH},
};

use image::GrayImage;
use tokio::sync::mpsc;

use crate::{
	cam::FrameResult,
	config::{CameraConfig, RuntimeConfig},
	util::Timer,
};

use super::{CameraBackend, CameraFrame, CameraSetupError};

pub struct FakeCamera {}

impl CameraBackend for FakeCamera {
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
		frame_tx: mpsc::Sender<FrameResult>,
	) -> Result<Self, CameraSetupError> {
		let _ = runtime_config;

		let image = image::open(&config.device_id)
			.map_err(|e| CameraSetupError::CameraNotFound(e.to_string()))?;
		let image = Arc::new(GrayImage::from(image));

		let mut timer = Timer::new();
		let interval = Duration::from_secs_f32(1.0 / config.fps as f32);
		let update_rate = interval.div_f32(2.0);

		tokio::spawn(async move {
			loop {
				if timer.interval(interval) {
					let timestamp = SystemTime::now()
						.duration_since(UNIX_EPOCH)
						.unwrap_or_default()
						.as_secs_f64();

					let _ = frame_tx.try_send(Ok(CameraFrame {
						timestamp,
						image: image.clone(),
					}));
				}

				tokio::time::sleep(update_rate).await;
			}
		});

		Ok(Self {})
	}
}
