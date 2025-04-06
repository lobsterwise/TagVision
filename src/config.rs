use serde::Deserialize;

use crate::cv::apriltag::params::AprilTagDetectorParams;

/// Configuration for the whole program
#[derive(Deserialize)]
pub struct Config {
	/// Configuration for each of the modules
	#[serde(default)]
	pub modules: Vec<ModuleConfig>,
	/// Parameters for the AprilTag detector
	#[serde(default)]
	pub detector_params: AprilTagDetectorParams,
}

/// Configuration for a single camera module
#[derive(Deserialize)]
pub struct ModuleConfig {
	/// Configuration for the module's camera
	pub camera: CameraConfig,
}

/// Configuration for a module's camera
#[derive(Deserialize)]
pub struct CameraConfig {
	/// The device ID of this module's camera
	pub device_id: u8,
	/// The resolution width of the camera in pixels
	pub width: u16,
	/// The resolution height of the camera in pixels
	pub height: u16,
	/// The desired frame rate of the camera in frames per second
	pub fps: u8,
	/// The exposure of the camera in microseconds of exposure time
	#[serde(default)]
	pub exposure: Option<u16>,
	/// How much to brighten the input frames, which can allow you to still detect tags with a lower exposure
	#[serde(default)]
	pub brightness: Option<f32>,
	/// How much to adjust the contrast of the input frames
	#[serde(default)]
	pub contrast: Option<f32>,
}

/// Camera intrinsics
#[derive(Deserialize)]
pub struct CameraCalibration {
	pub camera_matrix: [[f64; 3]; 3],
	pub distortion: [f64; 5],
}
