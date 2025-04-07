use std::{
	sync::Arc,
	time::{SystemTime, UNIX_EPOCH},
};

use crossbeam_queue::ArrayQueue;
use image::GrayImage;

use crate::config::{CameraConfig, RuntimeConfig, DEFAULT_QUEUE_SIZE};

use super::{CameraBackend, CameraFrame, CameraSetupError, CaptureError};

pub struct UVCCamera {
	queue: Arc<ArrayQueue<Result<CameraFrame, CaptureError>>>,
}

impl CameraBackend for UVCCamera {
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError> {
		let context =
			uvc::Context::new().map_err(|e| CameraSetupError::LibraryInitError(e.to_string()))?;

		let format = uvc::StreamFormat {
			width: config.width as u32,
			height: config.height as u32,
			fps: config.fps as u32,
			format: uvc::FrameFormat::MJPEG,
		};

		let device = context
			.find_device(None, None, Some(&config.device_id))
			.map_err(|e| CameraSetupError::CameraNotFound(e.to_string()))?;

		let device_handle = device
			.open()
			.map_err(|e| CameraSetupError::StartError(e.to_string()))?;

		let mut stream_handle = device_handle
			.get_stream_handle_with_format(format)
			.map_err(|e| CameraSetupError::GeneralError(e.to_string()))?;

		let queue = Arc::new(ArrayQueue::new(
			runtime_config
				.camera_queue_size
				.unwrap_or(DEFAULT_QUEUE_SIZE) as usize,
		));

		stream_handle
			.start_stream(
				|frame, queue| {
					let timestamp = SystemTime::now()
						.duration_since(UNIX_EPOCH)
						.unwrap_or_default()
						.as_micros();
					// Avoid the copy here
					// let data = unsafe {
					// 	let data = frame.to_bytes().as_ptr();
					// 	// Avoid the drop freeing the frame twice
					// 	std::mem::forget(frame);
					// 	let data = std::slice::from_ref(s)
					// };
					let image = GrayImage::from_raw(
						frame.width(),
						frame.height(),
						frame.to_bytes().to_vec(),
					);
					if let Some(image) = image {
						queue.force_push(Ok(CameraFrame { timestamp, image }));
					} else {
						queue.force_push(Err(CaptureError::DecodeError(
							"Improper dimensions".into(),
						)));
					}
				},
				queue.clone(),
			)
			.map_err(|e| CameraSetupError::StartError(e.to_string()))?;

		Ok(Self { queue })
	}

	fn get_frames(&mut self) -> Result<Vec<Result<CameraFrame, CaptureError>>, CaptureError> {
		let mut out = Vec::new();
		while let Some(frame) = self.queue.pop() {
			out.push(frame);
		}

		Ok(out)
	}
}
