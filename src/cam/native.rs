use std::{
	sync::Arc,
	time::{SystemTime, UNIX_EPOCH},
};

use nokhwa::{
	pixel_format::LumaFormat,
	utils::{
		ApiBackend, CameraFormat, CameraIndex, ControlValueSetter, FrameFormat, KnownCameraControl,
		RequestedFormat, RequestedFormatType, Resolution,
	},
	Camera, NokhwaError,
};
use tokio::sync::{mpsc, oneshot};

use crate::{
	cam::FrameResult,
	config::{CameraConfig, RuntimeConfig},
};

use super::{CameraBackend, CameraFrame, CameraSetupError, CaptureError};

pub struct NativeCamera {
	capture_error: oneshot::Receiver<CameraSetupError>,
}

impl CameraBackend for NativeCamera {
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
		frame_tx: mpsc::Sender<FrameResult>,
	) -> Result<Self, CameraSetupError> {
		let _ = runtime_config;

		// Camera setup
		let index = get_camera_index(config.device_id.clone())
			.map_err(|e| CameraSetupError::CameraNotFound(e.to_string()))?;

		let resolution = Resolution::new(config.width as u32, config.height as u32);

		let camera_format = CameraFormat::new(resolution, FrameFormat::MJPEG, config.fps as u32);
		let format = RequestedFormat::<'static>::new::<LumaFormat>(RequestedFormatType::Exact(
			camera_format,
		));

		#[cfg(target_os = "linux")]
		let backend = ApiBackend::Video4Linux;
		#[cfg(target_os = "windows")]
		let backend = ApiBackend::MediaFoundation;
		#[cfg(target_os = "macos")]
		let backend = ApiBackend::AVFoundation;

		// Setup camera thread
		let (error_tx, error_rx) = oneshot::channel::<CameraSetupError>();

		{
			let config = config.clone();
			tokio::task::spawn_blocking(move || {
				let camera = Camera::with_backend(index, format, backend);
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

					let _ = frame_tx.try_send(frame);
				}
			});
		}

		Ok(Self {
			capture_error: error_rx,
		})
	}

	fn self_check(&mut self) -> Option<CameraSetupError> {
		self.capture_error.try_recv().ok()
	}
}

/// Gets the final index of the camera from the given ID
fn get_camera_index(camera_id: String) -> std::io::Result<CameraIndex> {
	if let Ok(index) = camera_id.parse() {
		Ok(CameraIndex::Index(index))
	} else {
		#[cfg(target_os = "linux")]
		return super::lookup::lookup_camera_id_linux(&camera_id).map(CameraIndex::Index);
		#[cfg(not(target_os = "linux"))]
		return Ok(CameraIndex::String(camera_id.clone()));
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
