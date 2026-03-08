use std::{
	panic::catch_unwind,
	sync::Arc,
	time::{SystemTime, UNIX_EPOCH},
};

use nokhwa::{
	pixel_format::LumaFormat,
	utils::{
		ApiBackend, CameraFormat, CameraIndex, FrameFormat, RequestedFormat, RequestedFormatType,
		Resolution,
	},
	Camera, NokhwaError,
};
use tokio::sync::{mpsc, oneshot};
use tracing::{error, info};

use crate::{
	cam::{v4l::apply_v4l_conf, FrameResult},
	config::{CameraConfig, FrameFormatOption, RuntimeConfig},
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

		let frame_format = match config.frame_format {
			FrameFormatOption::MJPEG => FrameFormat::MJPEG,
			FrameFormatOption::YUYV => FrameFormat::YUYV,
		};

		let camera_format = CameraFormat::new(resolution, frame_format, config.fps as u32);
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
			std::thread::spawn(move || {
				// We have to configure before starting the camera or else we get permission errors
				if let Ok(index) = index.as_index() {
					if let Err(e) = apply_v4l_conf(index as u8, &config) {
						error!("Failed to configure camera: {e}");
					}
				}
				let camera = Camera::with_backend(index.clone(), format, backend);
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

				if let Err(e) = camera.open_stream() {
					let _ = error_tx.send(CameraSetupError::GeneralError(e.to_string()));
				}

				info!("Initialized native camera {index}");

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

						let frame = match frame_format {
							FrameFormat::MJPEG => turbojpeg::decompress_image(frame.buffer())
								.map_err(|e| CaptureError::DecodeError(e.to_string())),
							_ => {
								let frame = catch_unwind(|| frame.decode_image::<LumaFormat>());
								let frame = match frame {
									Ok(frame) => frame,
									Err(e) => {
										return Err(CaptureError::GeneralError(format!(
											"Decode panic: {:?}",
											e.downcast::<String>()
										)))
									}
								};

								frame.map_err(|e| CaptureError::DecodeError(e.to_string()))
							}
						};

						let frame = frame?;

						Ok(CameraFrame {
							image: Arc::new(frame),
							timestamp,
						})
					});

					if let Err(mpsc::error::TrySendError::Closed(..)) = frame_tx.try_send(frame) {
						break;
					}
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
		return super::v4l::lookup_camera_id_linux(&camera_id).map(CameraIndex::Index);
		#[cfg(not(target_os = "linux"))]
		return Ok(CameraIndex::String(camera_id.clone()));
	}
}
