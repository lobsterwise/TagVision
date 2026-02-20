use nalgebra::{Matrix3, Matrix3x1};

use super::apriltag::layout::LayoutPose;

#[derive(Clone, Debug, Default)]
pub struct Pose3D {
	pub x: f64,
	pub y: f64,
	pub z: f64,
	pub rx: f64,
	pub ry: f64,
	pub rz: f64,
}

impl Pose3D {
	pub fn from_matrices(translation: Matrix3x1<f64>, rotation_matrix: Matrix3<f64>) -> Self {
		let (rx, ry, rz) = rotation_matrix_to_euler(rotation_matrix);

		Self {
			x: translation[(0, 0)],
			y: translation[(1, 0)],
			z: translation[(2, 0)],
			rx,
			ry,
			rz,
		}
	}

	pub fn from_tag_pose(pose: LayoutPose) -> Self {
		let qw = pose.rotation.quaternion.w;
		let qx = pose.rotation.quaternion.x;
		let qy = pose.rotation.quaternion.y;
		let qz = pose.rotation.quaternion.z;
		let qx2 = qx * qx;
		let qy2 = qy * qy;
		let qz2 = qz * qz;
		let rotation_matrix = nalgebra::matrix![
			1.0 - 2.0 * (qy2 + qz2), 2.0 * (qx * qy - qw * qz), 2.0 * (qw * qz + qx * qz);
			2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx2 + qz2), 2.0 * (qy * qz - qw * qx);
			2.0 * (qx * qz - qw * qy), 2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx2 + qy2)
		];

		let translation_matrix =
			nalgebra::matrix![pose.translation.x; pose.translation.y; pose.translation.z];

		Self::from_matrices(translation_matrix, rotation_matrix)
	}

	pub fn get_rotation_matrix(&self) -> Matrix3<f64> {
		// euler_to_rotation_matrix(self.rx, self.ry, self.rz)
		nalgebra::Rotation3::from_euler_angles(self.rx, self.ry, self.rz).into()
	}

	pub fn add(&self, other: &Self) -> Self {
		Self {
			x: self.x + other.x,
			y: self.y + other.y,
			z: self.z + other.z,
			rx: self.rx + other.rx,
			ry: self.ry + other.ry,
			rz: self.rz + other.rz,
		}
	}
}

fn rotation_matrix_to_euler(rotation_matrix: Matrix3<f64>) -> (f64, f64, f64) {
	let sy = (rotation_matrix[(0, 0)].powi(2) + rotation_matrix[(1, 0)].powi(2)).sqrt();

	if sy >= 1.0e-6 {
		(
			rotation_matrix[(2, 1)].atan2(rotation_matrix[(2, 2)]),
			(-rotation_matrix[(2, 0)]).atan2(sy),
			rotation_matrix[(1, 0)].atan2(rotation_matrix[(0, 0)]),
		)
	} else {
		(
			(-rotation_matrix[(2, 1)]).atan2(rotation_matrix[(2, 2)]),
			(-rotation_matrix[(2, 0)]).atan2(sy),
			0.0,
		)
	}
}

fn euler_to_rotation_matrix(rx: f64, ry: f64, rz: f64) -> Matrix3<f64> {
	let x_matrix = nalgebra::matrix![
		1.0, 0.0, 0.0;
		0.0, rx.cos(), -rx.sin();
		0.0, rx.sin(), rx.cos()
	];
	let y_matrix = nalgebra::matrix![
		ry.cos(), 0.0, ry.sin();
		0.0, 1.0, 0.0;
		-ry.sin(), 0.0, ry.cos()
	];
	let z_matrix = nalgebra::matrix![
		rz.cos(), -rz.sin(), 0.0;
		rz.sin(), rz.cos(), 0.0;
		0.0, 0.0, 1.0;
	];

	x_matrix * y_matrix * z_matrix
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
	Multi(Vec<Pose3DWithError>),
}

/// A timestamped PnP solution
#[derive(Clone, Debug)]
pub struct PoseUpdate {
	pub pose: Pose3D,
	pub timestamp: u128,
}

impl PoseUpdate {
	pub fn as_array(&self) -> [f64; 7] {
		[
			self.pose.x,
			self.pose.y,
			self.pose.z,
			self.pose.rx,
			self.pose.ry,
			self.pose.rz,
			self.timestamp as f64,
		]
	}
}
