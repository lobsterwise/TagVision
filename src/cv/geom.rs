use nalgebra::{Isometry3, Matrix3, Matrix3x1, OPoint, Translation3, UnitQuaternion, Vector3};

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

	pub fn wpi_to_transform(&self) -> Pose3D {
		let mut pose = self.clone();
		pose.t = pose.r * pose.t;
		pose
	}

	pub fn to_wpi(&self) -> Pose3D {
		let mut pose = self.clone();
		pose.t = pose.r.try_inverse().unwrap() * pose.t;
		pose
	}
}

/// Changes an isometry of rotation + translation in rotated frame to a translation then a rotation
pub fn reverse_isometry(iso: Isometry3<f64>) -> (Vector3<f64>, UnitQuaternion<f64>) {
	let translation = iso.transform_point(&OPoint::origin()).coords;
	let rotation = iso.rotation;

	(translation, rotation)
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
	pub timestamp: u128,
}
