use std::sync::Arc;

use fake::FakeCamera;
use image::GrayImage;

use crate::config::{CameraBackendOption, CameraConfig, RuntimeConfig};

/// A fake camera that produces a single static image
pub mod fake;
/// GStreamer camera capture
#[cfg(feature = "gstreamer")]
pub mod gstreamer;
/// Native camera capture using nokhwa
#[cfg(feature = "native")]
pub mod native;
/// Bare UVC camera capture
#[cfg(feature = "uvc")]
pub mod uvc;

/// A camera that implements a backend
pub struct Camera {
	backend: Box<dyn CameraBackend>,
	frame_buffer: Vec<Result<CameraFrame, CaptureError>>,
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
		Ok(Self {
			backend: get_backend(&config, runtime_config)?,
			frame_buffer: Vec::new(),
			frame_error_count: 0,
			max_frame_errors,
		})
	}

	/// Gets the next available frames from the camera
	pub fn get_frames(&mut self) -> Result<&Vec<Result<CameraFrame, CaptureError>>, CaptureError> {
		self.frame_buffer.clear();
		self.backend.get_frames(&mut self.frame_buffer)?;

		for result in &self.frame_buffer {
			if result.is_err() {
				self.frame_error_count += 1;
			} else {
				self.frame_error_count = 0;
			}
		}

		Ok(&self.frame_buffer)
	}

	/// Checks this camera and returns whether it needs a restart
	pub fn self_check(&mut self) -> bool {
		if self.frame_error_count > self.max_frame_errors {
			true
		} else {
			if let Some(err) = self.backend.self_check() {
				eprintln!("Camera error: {err}");
				true
			} else {
				false
			}
		}
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

	/// Gets all new frames from the camera. The provided buffer will be empty.
	fn get_frames(
		&mut self,
		buf: &mut Vec<Result<CameraFrame, CaptureError>>,
	) -> Result<(), CaptureError>;

	/// Checks for an extra setup error if available to automatically restart the camera
	fn self_check(&mut self) -> Option<CameraSetupError> {
		None
	}
}

/// A single timestamped camera frame
#[derive(Clone)]
pub struct CameraFrame {
	pub timestamp: u128,
	pub image: Arc<GrayImage>,
}

fn get_backend(
	config: &CameraConfig,
	runtime_config: &RuntimeConfig,
) -> Result<Box<dyn CameraBackend>, CameraSetupError> {
	match config.backend {
		CameraBackendOption::Native => {
			#[cfg(feature = "native")]
			return Ok(Box::new(native::NativeCamera::init(
				config,
				runtime_config,
			)?));
			#[cfg(not(feature = "native"))]
			return Err(CameraSetupError::BackendMissing);
		}
		CameraBackendOption::UVC => {
			#[cfg(feature = "uvc")]
			return Ok(Box::new(uvc::UVCCamera::init(config, runtime_config)?));
			#[cfg(not(feature = "uvc"))]
			return Err(CameraSetupError::BackendMissing);
		}
		CameraBackendOption::GStreamer => {
			#[cfg(feature = "gstreamer")]
			return Ok(Box::new(gstreamer::GStreamerCamera::init(
				config,
				runtime_config,
			)?));
			#[cfg(not(feature = "gstreamer"))]
			return Err(CameraSetupError::BackendMissing);
		}
		CameraBackendOption::Fake => {
			return Ok(Box::new(FakeCamera::init(config, runtime_config)?))
		}
	}
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
