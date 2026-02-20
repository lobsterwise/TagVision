use std::{
	sync::Arc,
	time::{SystemTime, UNIX_EPOCH},
};

use crossbeam_queue::ArrayQueue;
use nokhwa::{
	pixel_format::LumaFormat,
	utils::{
		CameraFormat, CameraIndex, ControlValueSetter, FrameFormat, KnownCameraControl,
		RequestedFormat, RequestedFormatType, Resolution,
	},
	Camera, NokhwaError,
};
use tokio::sync::oneshot;

use crate::config::{CameraConfig, RuntimeConfig, DEFAULT_QUEUE_SIZE};

use super::{CameraBackend, CameraFrame, CameraSetupError, CaptureError};

pub struct NativeCamera {
	queue: Arc<ArrayQueue<Result<CameraFrame, CaptureError>>>,
	capture_error: oneshot::Receiver<CameraSetupError>,
}

impl CameraBackend for NativeCamera {
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError> {
		// Camera setup
		let index = if let Ok(index) = config.device_id.parse() {
			CameraIndex::Index(index)
		} else {
			CameraIndex::String(config.device_id.clone())
		};

		let resolution = Resolution::new(config.width as u32, config.height as u32);

		let camera_format = CameraFormat::new(resolution, FrameFormat::MJPEG, config.fps as u32);
		let format = RequestedFormat::<'static>::new::<LumaFormat>(RequestedFormatType::Exact(
			camera_format,
		));

		// Setup queue and camera thread
		let queue = Arc::new(ArrayQueue::new(
			runtime_config
				.camera_queue_size
				.unwrap_or(DEFAULT_QUEUE_SIZE) as usize,
		));

		let (error_tx, error_rx) = oneshot::channel::<CameraSetupError>();

		{
			let config = config.clone();
			let queue = queue.clone();
			tokio::spawn(async move {
				let camera = Camera::new(index, format);
				let mut camera = match camera {
					Ok(camera) => camera,
					Err(e) => {
						match e {
							e @ NokhwaError::OpenDeviceError(..) => {
								let _ =
									error_tx.send(CameraSetupError::CameraNotFound(e.to_string()));
							}
							other => {
								let _ = error_tx
									.send(CameraSetupError::GeneralError(other.to_string()));
							}
						}
						return;
					}
				};

				// Configuration
				if let Some(exposure) = config.exposure {
					if let Err(e) = configure_camera(
						&mut camera,
						KnownCameraControl::Exposure,
						ControlValueSetter::Integer(exposure as i64),
					) {
						let _ = error_tx.send(e);
						return;
					}
				}

				if !config.manual_brightness {
					if let Some(brightness) = config.brightness {
						if let Err(e) = configure_camera(
							&mut camera,
							KnownCameraControl::Brightness,
							ControlValueSetter::Float(brightness as f64),
						) {
							let _ = error_tx.send(e);
							return;
						}
					}
				}

				if !config.manual_contrast {
					if let Some(contrast) = config.contrast {
						if let Err(e) = configure_camera(
							&mut camera,
							KnownCameraControl::Brightness,
							ControlValueSetter::Float(contrast as f64),
						) {
							let _ = error_tx.send(e);
							return;
						}
					}
				}

				if let Err(e) = camera.open_stream() {
					let _ = error_tx.send(CameraSetupError::GeneralError(e.to_string()));
					return;
				}

				loop {
					let frame = match camera.frame() {
						Ok(frame) => Ok(frame),
						Err(e) => match e {
							NokhwaError::ReadFrameError(e) => {
								Err(CaptureError::GeneralError(e.to_string()))
							}
							e @ NokhwaError::ProcessFrameError { .. } => {
								Err(CaptureError::DecodeError(e.to_string()))
							}
							other => Err(CaptureError::GeneralError(other.to_string())),
						},
					};

					let frame = frame.and_then(|frame| {
						let timestamp = SystemTime::now()
							.duration_since(UNIX_EPOCH)
							.unwrap_or_default()
							.as_millis();

						let frame = match frame.decode_image::<LumaFormat>() {
							Ok(frame) => frame,
							Err(e) => return Err(CaptureError::DecodeError(e.to_string())),
						};

						Ok(CameraFrame {
							image: Arc::new(frame),
							timestamp,
						})
					});

					let _ = queue.push(frame);
				}
			});
		}

		Ok(Self {
			queue,
			capture_error: error_rx,
		})
	}

	fn get_frames(
		&mut self,
		buf: &mut Vec<Result<CameraFrame, CaptureError>>,
	) -> Result<(), CaptureError> {
		while let Some(frame) = self.queue.pop() {
			buf.push(frame);
		}

		Ok(())
	}

	fn self_check(&mut self) -> Option<CameraSetupError> {
		self.capture_error.try_recv().ok()
	}
}

fn configure_camera(
	camera: &mut Camera,
	id: KnownCameraControl,
	value: ControlValueSetter,
) -> Result<(), CameraSetupError> {
	match camera.set_camera_control(id, value) {
		Ok(..) => Ok(()),
		Err(e) => match e {
			NokhwaError::SetPropertyError {
				property,
				value: _,
				error,
			} => Err(CameraSetupError::ConfigError(format!(
				"Failed to configure {property}: {error}"
			))),
			other => Err(CameraSetupError::GeneralError(other.to_string())),
		},
	}
}
