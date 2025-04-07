#[derive(Clone, Debug, Default)]
pub struct Pose3D {
	pub x: f64,
	pub y: f64,
	pub z: f64,
	pub rx: f64,
	pub ry: f64,
	pub rz: f64,
}

/// A 3D pose with a reprojection error
#[derive(Clone, Debug, Default)]
pub struct Pose3DWithError {
	pub pose: Pose3D,
	pub error: f64,
}

/// Solution for solvePnp, either one pose or two
#[derive(Clone, Debug)]
pub enum PnPSolution {
	Single(Pose3DWithError),
	Double(Pose3DWithError, Pose3DWithError),
}

/// A timestamped PnP solution
#[derive(Clone, Debug)]
pub struct PoseUpdate {
	pub pose: PnPSolution,
	pub timestamp: u128,
}
