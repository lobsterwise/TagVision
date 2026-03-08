use std::sync::Arc;

use fake::FakeCamera;
use image::GrayImage;
use tokio::sync::mpsc;

use crate::config::{CameraBackendOption, CameraConfig, RuntimeConfig};

/// A fake camera that produces a single static image
pub mod fake;
/// GStreamer camera capture
#[cfg(feature = "gstreamer")]
pub mod gstreamer;
/// Native camera capture using nokhwa
#[cfg(feature = "native")]
pub mod native;
/// Camera lookup
#[cfg(target_os = "linux")]
mod v4l;

/// A camera that implements a backend
pub struct Camera {
	backend: Box<dyn CameraBackend + Send + Sync>,
	frame_rx: mpsc::Receiver<FrameResult>,
	/// Number of errors in consecutive frames since the last restart
	frame_error_count: u32,
	max_frame_errors: u32,
}

impl Camera {
	/// Initializes this camera from configuration
	pub fn from_config(
		config: CameraConfig,
		runtime_config: &RuntimeConfig,
		max_frame_errors: u32,
	) -> Result<Self, CameraSetupError> {
		let (tx, rx) = mpsc::channel(1);

		Ok(Self {
			backend: get_backend(&config, runtime_config, tx)?,
			frame_rx: rx,
			frame_error_count: 0,
			max_frame_errors,
		})
	}

	/// Gets the next available frame from the camera
	pub async fn get_frame(&mut self) -> FrameResult {
		match self.frame_rx.recv().await {
			Some(result) => {
				if result.is_err() {
					self.frame_error_count += 1;
				}

				result
			}
			None => {
				self.frame_error_count += 1;
				Err(CaptureError::PipelineError("Frame queue closed".into()))
			}
		}
	}

	/// Checks this camera and returns whether it needs a restart
	pub fn self_check(&mut self) -> Option<CameraSetupError> {
		if self.frame_error_count > self.max_frame_errors {
			self.frame_error_count = 0;
			Some(CameraSetupError::TooManyFrameErrors)
		} else {
			self.backend.self_check()
		}
	}
}

/// Trait for a camera backend that returns images
pub trait CameraBackend {
	/// Creates a new camera with the given parameters
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
		frame_tx: mpsc::Sender<FrameResult>,
	) -> Result<Self, CameraSetupError>
	where
		Self: Sized;

	/// Checks for an extra setup error if available to automatically restart the camera
	fn self_check(&mut self) -> Option<CameraSetupError> {
		None
	}
}

/// A single timestamped camera frame
#[derive(Clone)]
pub struct CameraFrame {
	/// Timestamp for the frame in milliseconds
	pub timestamp: u128,
	pub image: Arc<GrayImage>,
}

fn get_backend(
	config: &CameraConfig,
	runtime_config: &RuntimeConfig,
	frame_tx: mpsc::Sender<FrameResult>,
) -> Result<Box<dyn CameraBackend + Send + Sync>, CameraSetupError> {
	match config.backend {
		CameraBackendOption::Native => {
			#[cfg(feature = "native")]
			return Ok(Box::new(native::NativeCamera::init(
				config,
				runtime_config,
				frame_tx,
			)?));
			#[cfg(not(feature = "native"))]
			return Err(CameraSetupError::BackendMissing);
		}
		CameraBackendOption::GStreamer => {
			#[cfg(feature = "gstreamer")]
			return Ok(Box::new(gstreamer::GStreamerCamera::init(
				config,
				runtime_config,
				frame_tx,
			)?));
			#[cfg(not(feature = "gstreamer"))]
			return Err(CameraSetupError::BackendMissing);
		}
		CameraBackendOption::Fake => {
			return Ok(Box::new(FakeCamera::init(
				config,
				runtime_config,
				frame_tx,
			)?))
		}
	}
}

/// A single result for a camera frame
pub type FrameResult = Result<CameraFrame, CaptureError>;

/// Different errors for camera creation
#[derive(thiserror::Error, Debug, Clone)]
#[allow(dead_code)]
pub enum CameraSetupError {
	/// The given backend was not compiled in the binary
	#[allow(dead_code)]
	#[error("Backend was not compiled in binary")]
	BackendMissing,
	/// Error initializing the capture library
	#[error("Failed to initialize capture library: {0}")]
	LibraryInitError(String),
	/// The camera could not be found
	#[error("Camera not found: {0}")]
	CameraNotFound(String),
	/// The camera configuration was invalid
	#[error("Invalid configuration: {0}")]
	InvalidConfigError(String),
	/// Configuration for the camera failed to apply
	#[error("Failed to configure camera: {0}")]
	ConfigError(String),
	/// Failed to start streaming data from the camera
	#[error("Failed to start camera: {0}")]
	StartError(String),
	/// Too many errors from frames
	#[error("Too many frame errors")]
	TooManyFrameErrors,
	/// General camera setup error
	#[error("General setup error: {0}")]
	GeneralError(String),
}

/// Different errors for frame capture
#[derive(thiserror::Error, Debug)]
pub enum CaptureError {
	/// The frame failed to decode to the proper format
	#[error("Failed to decode frame: {0}")]
	DecodeError(String),
	/// Queue / pipeline error
	#[error("Frame pipeline failed: {0}")]
	PipelineError(String),
	/// General capture error
	#[error("General capture error: {0}")]
	GeneralError(String),
}
