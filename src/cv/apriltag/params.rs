use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct AprilTagDetectorParams {
	/// The number of threads to use for the detector
	pub thread_count: u8,
	/// How much to decimate the pixels of the input.
	/// Values above 2.0 can massively increase speed at the cost of some accuracy and detection range.
	/// Generally the speed will only change when the parameter moves between whole numbers. (i.e. 2.0 will be much faster than 1.99)
	pub quad_decimate: f32,
}

impl Default for AprilTagDetectorParams {
	fn default() -> Self {
		Self {
			thread_count: 1,
			quad_decimate: 2.0,
		}
	}
}
