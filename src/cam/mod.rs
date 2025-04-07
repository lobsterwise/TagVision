use fake::FakeCamera;
use gstreamer::GStreamerCamera;
use image::GrayImage;
use uvc::UVCCamera;

use crate::config::{CameraBackendOption, CameraConfig, RuntimeConfig};

/// A fake camera that produces a single static image
pub mod fake;
/// GStreamer camera capture
#[cfg(feature = "gstreamer")]
pub mod gstreamer;
/// Bare UVC camera capture
#[cfg(feature = "uvc")]
pub mod uvc;

/// A camera that implements a backend
pub struct Camera {
	backend: Box<dyn CameraBackend>,
	config: CameraConfig,
}

impl Camera {
	pub fn from_config(
		config: CameraConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError> {
		let backend: Box<dyn CameraBackend> = match config.backend {
			CameraBackendOption::UVC => {
				if cfg!(feature = "uvc") {
					#[cfg(feature = "uvc")]
					Box::new(UVCCamera::init(&config, runtime_config)?)
				} else {
					return Err(CameraSetupError::BackendMissing);
				}
			}
			CameraBackendOption::GStreamer => {
				if cfg!(feature = "gstreamer") {
					#[cfg(feature = "gstreamer")]
					Box::new(GStreamerCamera::init(&config, runtime_config)?)
				} else {
					return Err(CameraSetupError::BackendMissing);
				}
			}
			CameraBackendOption::Fake => Box::new(FakeCamera::init(&config, runtime_config)?),
		};

		Ok(Self { backend, config })
	}

	pub fn get_frames(&mut self) -> Result<Vec<Result<CameraFrame, CaptureError>>, CaptureError> {
		self.backend.get_frames()
	}
}

/// Trait for a camera backend that returns images
pub trait CameraBackend {
	/// Creates a new camera with the given parameters
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError>
	where
		Self: Sized;

	/// Gets all new frames from the camera
	fn get_frames(&mut self) -> Result<Vec<Result<CameraFrame, CaptureError>>, CaptureError>;
}

/// A single timestamped camera frame
pub struct CameraFrame {
	pub timestamp: u128,
	pub image: GrayImage,
}

/// Different errors for camera creation
#[derive(thiserror::Error, Debug)]
pub enum CameraSetupError {
	/// The given backend was not compiled in the binary
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
	/// General capture error
	#[error("General capture error: {0}")]
	GeneralError(String),
}
