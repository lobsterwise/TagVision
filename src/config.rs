use std::collections::HashMap;

use serde::Deserialize;

use crate::cv::{apriltag::{layout::AprilTagLayoutPreset, params::AprilTagDetectorParams}, distort::CameraCalibration};

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
	/// Configuration for the runtime
	#[serde(default)]
	pub runtime: RuntimeConfig,
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
	/// Time to wait between reconnects, in seconds
	#[serde(default)]
	pub reconnect_time: Option<f32>,
}

/// Configuration for the tags
#[derive(Deserialize, Clone)]
pub struct TagConfig {
	/// The size of one side of the AprilTags in inches
	pub tag_size: f64,
	/// The name of the tag layout to use
	pub layout: AprilTagLayoutPreset,
}

/// Configuration for a single camera module
#[derive(Deserialize, Clone)]
pub struct ModuleConfig {
	/// The ID for the module, used to identify it in logs and network tables
	pub id: String,
	/// Configuration for the module's camera
	pub camera: CameraConfig,
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
	/// Whether or not this is a monochrome camera. If it is, we can skip the conversion to monochrome for the AprilTag detector
	pub is_monochrome: bool,
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
	#[serde(rename = "uvc")]
	#[default]
	UVC,
	#[serde(rename = "gstreamer")]
	GStreamer,
	#[serde(rename = "fake")]
	Fake,
}

/// Configuration for the runtime
#[derive(Deserialize, Default, Clone)]
pub struct RuntimeConfig {
	/// Specifies how many frames to keep in the camera frame queue before they start to be discarded
	#[serde(default)]
	pub camera_queue_size: Option<u8>,
	/// How often to update the runtime, in milliseconds
	#[serde(default)]
	pub update_rate: Option<f32>,
	/// How often to reconnect cameras that have come disconnected, in seconds
	#[serde(default)]
	pub reconnect_interval: Option<f32>,
}

pub const DEFAULT_QUEUE_SIZE: u8 = 10;
pub const DEFAULT_UPDATE_RATE: f32 = 0.01;
pub const DEFAULT_RECONNECT_INTERVAL: f32 = 1.0;
