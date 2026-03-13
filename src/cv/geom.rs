use nalgebra::{Isometry3, Matrix3, Matrix3x1, Matrix6, Translation3, UnitQuaternion, Vector3};

use super::apriltag::layout::LayoutPose;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Pose3D {
	// Translation
	pub t: Vector3<f64>,
	// Rotation matrix
	pub r: Matrix3<f64>,
}

impl Pose3D {
	pub fn from_matrices(translation: Matrix3x1<f64>, rotation_matrix: Matrix3<f64>) -> Self {
		Self {
			t: translation,
			r: rotation_matrix,
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
			1.0 - 2.0*(qy2 + qz2),     2.0*(qx*qy - qw*qz),     2.0*(qx*qz + qw*qy);
			2.0*(qx*qy + qw*qz),       1.0 - 2.0*(qx2 + qz2),   2.0*(qy*qz - qw*qx);
			2.0*(qx*qz - qw*qy),       2.0*(qy*qz + qw*qx),     1.0 - 2.0*(qx2 + qy2)
		];

		let translation_matrix =
			nalgebra::matrix![pose.translation.x; pose.translation.y; pose.translation.z];

		Self::from_matrices(translation_matrix, rotation_matrix)
	}

	pub fn add(&self, other: &Self) -> Self {
		Self {
			t: self.t + other.t,
			r: self.r * other.r,
		}
	}

	pub fn to_isometry(&self) -> Isometry3<f64> {
		Isometry3::from_parts(
			Translation3::from(self.t),
			UnitQuaternion::from_matrix(&self.r),
		)
	}

	pub fn from_isometry(isometry: Isometry3<f64>) -> Self {
		Self {
			t: isometry.translation.vector,
			r: isometry.rotation.to_rotation_matrix().matrix().clone(),
		}
	}

	pub fn inverse(&self) -> Self {
		Self {
			t: -1.0 * self.t,
			r: self.r.transpose(),
		}
	}
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
	Multi(Vec<Pose3DWithError>),
}

/// A timestamped PnP solution
#[derive(Clone, Debug)]
pub struct PoseUpdate {
	pub pose: Pose3DWithError,
	pub timestamp: f64,
	pub tag: u8,
	pub covariance: PoseCovariance,
}

#[derive(Clone, Debug)]
pub struct PoseCovariance {
	pub x: f64,
	pub y: f64,
	pub z: f64,
	pub rx: f64,
	pub ry: f64,
	pub rz: f64,
}

impl PoseCovariance {
	pub fn from_matrix(mat: Matrix6<f64>) -> Self {
		Self {
			rx: mat[(0, 0)].sqrt(),
			ry: mat[(1, 1)].sqrt(),
			rz: mat[(2, 2)].sqrt(),
			x: mat[(3, 3)].sqrt(),
			y: mat[(4, 4)].sqrt(),
			z: mat[(5, 5)].sqrt(),
		}
	}
}
