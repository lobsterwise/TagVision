use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub struct AprilTagDetectorParams {
	/// The number of threads to use for the detector
	#[serde(default = "default_thread_count")]
	pub thread_count: u8,
	/// How much to decimate the pixels of the input.
	/// Values above 2.0 can massively increase speed at the cost of some accuracy and detection range.
	/// Generally the speed will only change when the parameter moves between whole numbers. (i.e. 2.0 will be much faster than 1.99)
	#[serde(default = "default_quad_decimate")]
	pub quad_decimate: f32,
	/// Whether to enable time profile reporting
	#[serde(default)]
	pub time_profile: bool,
}

impl Default for AprilTagDetectorParams {
	fn default() -> Self {
		Self {
			thread_count: default_thread_count(),
			quad_decimate: default_quad_decimate(),
			time_profile: false,
		}
	}
}

fn default_thread_count() -> u8 {
	2
}

fn default_quad_decimate() -> f32 {
	2.0
}
