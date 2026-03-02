use std::collections::HashMap;

use serde::Deserialize;

use crate::cv::{
	apriltag::{layout::AprilTagLayoutPreset, params::AprilTagDetectorParams},
	distort::CameraCalibration,
};

/// Configuration for the whole program
#[derive(Deserialize)]
pub struct Config {
	/// Configuration for the network
	pub network: NetworkConfig,
	/// Configuration for each of the modules
	#[serde(default)]
	pub modules: HashMap<String, ModuleConfig>,
	/// Parameters for the AprilTag detector
	#[serde(default)]
	pub detector_params: AprilTagDetectorParams,
	/// Configuration for the AprilTags
	pub tags: TagConfig,
	/// Tag filters
	#[serde(default)]
	pub filters: TagFilters,
	/// Configuration for the runtime
	#[serde(default)]
	pub runtime: RuntimeConfig,
	/// Pose estimator
	#[serde(default)]
	pub pose_estimator: PoseEstimatorOption,
}


/// Different backend implementations for pose estimation
#[derive(Deserialize, Default, Clone, Copy)]
pub enum PoseEstimatorOption {
	#[serde(rename = "homography")]
	#[default]
	Homography,
	#[serde(rename = "p3p")]
	P3P,
}

/// Configuration for the network
#[derive(Deserialize, Clone)]
pub struct NetworkConfig {
	/// The name of this TagVision runtime on the network
	pub name: String,
	/// The team number to use
	#[serde(default)]
	pub team_number: Option<u16>,
	/// The IP address to use
	#[serde(default)]
	pub address: Option<String>,
	/// Whether NT connection is disabled. Only for local testing.
	#[serde(default)]
	pub disabled: bool,
	/// Time to wait between reconnects, in seconds
	#[serde(default = "default_reconnect_interval")]
	pub reconnect_interval: f32,
}

fn default_reconnect_interval() -> f32 {
	2.0
}

/// Configuration for the tags
#[derive(Deserialize, Clone)]
pub struct TagConfig {
	/// The size of one side of the AprilTags in inches
	pub tag_size: f64,
	/// The name of the tag layout to use
	pub layout: AprilTagLayoutPreset,
}

impl TagConfig {
	/// Gets the size of one side of the AprilTags in meters
	pub fn tag_size_meters(&self) -> f64 {
		self.tag_size * 0.0254
	}
}

/// Filters for tag detections
#[derive(Deserialize, Clone, Default)]
pub struct TagFilters {
	/// Minimum area for detections
	pub min_area: Option<f64>,
	/// Maximum area for detections
	pub max_area: Option<f64>,
	/// Minimum perimeter for detections
	pub min_perimeter: Option<f64>,
	/// Maximum perimeter for detections
	pub max_perimeter: Option<f64>,
}

/// Configuration for a single camera module
#[derive(Deserialize, Clone)]
pub struct ModuleConfig {
	/// Configuration for the module's camera
	pub camera: CameraConfig,
	/// Maximum number of frame errors before the camera is restarted
	#[serde(default = "default_max_errors")]
	pub max_errors: u32,
	/// Whether to temporarily disable this module, for testing
	#[serde(default)]
	pub disabled: bool,
}

fn default_max_errors() -> u32 {
	5
}

/// Configuration for a module's camera
#[derive(Deserialize, Clone)]
pub struct CameraConfig {
	/// The device ID of this module's camera. Format depends on the camera backend used, but it's usually either an ID or a bus number
	pub device_id: String,
	/// The backend for this camera
	#[serde(default)]
	pub backend: CameraBackendOption,
	/// The resolution width of the camera in pixels
	pub width: u16,
	/// The resolution height of the camera in pixels
	pub height: u16,
	/// The desired frame rate of the camera in frames per second
	pub fps: u16,
	/// The intrinsics of the camera
	pub intrinsics: CameraCalibration,
	/// The exposure of the camera in microseconds of exposure time
	#[serde(default)]
	pub exposure: Option<u16>,
	/// How much to brighten the input frames, which can allow you to still detect tags with a lower exposure
	#[serde(default)]
	pub brightness: Option<f32>,
	/// How much to adjust the contrast of the input frames
	#[serde(default)]
	pub contrast: Option<f32>,
	/// Whether to do manual brightness adjustment instead of the camera's builtin brightness setting
	#[serde(default)]
	pub manual_brightness: bool,
	/// Whether to do manual contrast adjustment instead of the camera's builtin contrast setting
	#[serde(default)]
	pub manual_contrast: bool,
}

/// Different backend implementations for cameras
#[derive(Deserialize, Default, Clone, Copy)]
pub enum CameraBackendOption {
	#[serde(rename = "native")]
	#[default]
	Native,
	#[serde(rename = "gstreamer")]
	GStreamer,
	#[serde(rename = "fake")]
	Fake,
}

/// Configuration for the runtime
#[derive(Deserialize, Clone)]
pub struct RuntimeConfig {
	/// Specifies how many frames to keep in the camera frame queue before they start to be discarded
	#[serde(default = "default_camera_queue_size")]
	pub camera_queue_size: u8,
	/// Number of threads to use for vision processing
	#[serde(default = "default_vision_threads")]
	pub vision_threads: u8,
	/// How often to reconnect cameras that have come disconnected, in seconds
	#[serde(default = "default_camera_reconnect_interval")]
	pub camera_reconnect_interval: f32,
}

impl Default for RuntimeConfig {
	fn default() -> Self {
		Self {
			camera_queue_size: default_camera_queue_size(),
			vision_threads: default_vision_threads(),
			camera_reconnect_interval: default_camera_reconnect_interval(),
		}
	}
}

fn default_camera_queue_size() -> u8 {
	10
}

fn default_vision_threads() -> u8 {
	4
}

fn default_camera_reconnect_interval() -> f32 {
	1.0
}
