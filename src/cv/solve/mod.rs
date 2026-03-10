use nalgebra::{matrix, ArrayStorage, Const, Matrix, Matrix3, Matrix3x4, Matrix6, Vector3};

use crate::cv::{
	apriltag::{layout::AprilTagLayout, AprilTagDetection},
	distort::OpenCVCameraIntrinsics,
	geom::{Pose3D, Pose3DWithError},
};

use super::geom::PnPSolution;

pub mod ippe;
pub mod p3p;
pub mod polynomial;

pub trait PnPSolver {
	fn new() -> Self
	where
		Self: Sized;

	fn solve(
		&mut self,
		layout: &AprilTagLayout,
		detection: &AprilTagDetection,
		intrinsics: &OpenCVCameraIntrinsics,
	) -> Option<PnPSolution>;
}

/// Solver using the AprilTag library's homography solver
pub struct AprilTagHomographySolver;

impl PnPSolver for AprilTagHomographySolver {
	fn new() -> Self
	where
		Self: Sized,
	{
		Self
	}

	fn solve(
		&mut self,
		layout: &AprilTagLayout,
		detection: &AprilTagDetection,
		intrinsics: &OpenCVCameraIntrinsics,
	) -> Option<PnPSolution> {
		// World origin -> tag position
		let world_to_tag = layout.get_tag_pose(detection.id)?;

		// If you were the tag facing outward from the wall, +X is to the left, +Y is down, and +Z points forward
		let Pose3DWithError {
			pose: camera_to_tag,
			error,
		} = detection.solve(intrinsics, layout.tag_size());

		// Remap / rotate axes into correct frame
		let mut tag_to_camera = camera_to_tag.inverse();
		let axis_remap = matrix![
			0.0, 0.0, -1.0;
			1.0, 0.0, 0.0;
			0.0, -1.0, 0.0
		];
		// Rotate to face the tag instead of away from it
		let face_tag = matrix![
			-1.0, 0.0, 0.0;
			0.0, -1.0, 0.0;
			0.0, 0.0, 1.0
		];
		tag_to_camera.r = axis_remap * tag_to_camera.r * axis_remap.transpose();
		tag_to_camera.r = tag_to_camera.r * face_tag;
		tag_to_camera.t = axis_remap * tag_to_camera.t;

		// Compute relative to tag
		let world_to_camera = Pose3D {
			r: world_to_tag.r * tag_to_camera.r,
			t: world_to_tag.t + world_to_tag.r * tag_to_camera.t,
		};

		let world_to_camera = Pose3DWithError {
			pose: world_to_camera,
			error,
		};

		Some(PnPSolution::Multi(vec![world_to_camera]))
	}
}

/// Compute pose covariance from pixel noise using first-order propagation.
/// Returns 6x6 covariance matrix (rotation[0:3], translation[3:6])
pub fn pose_covariance(
	r: &Matrix3<f64>,
	t: &Vector3<f64>,
	object_points: Matrix3x4<f64>,
	fx: f64,
	fy: f64,
	sigma_px: f64,
) -> Matrix6<f64> {
	let n = object_points.len();
	assert!(n >= 4);

	let mut j = Matrix::<f64, Const<8>, Const<6>, ArrayStorage<f64, 8, 6>>::zeros();

	for i in 0..4 {
		let pw = object_points.column(i);
		let pc = r * pw + t;
		let x = pc.x;
		let y = pc.y;
		let z = pc.z;

		let inv_z = 1.0 / z;
		let inv_z2 = inv_z * inv_z;

		// --- Translation derivatives ---
		let du_dtx = fx * inv_z;
		let du_dty = 0.0;
		let du_dtz = -fx * x * inv_z2;

		let dv_dtx = 0.0;
		let dv_dty = fy * inv_z;
		let dv_dtz = -fy * y * inv_z2;

		// --- Rotation derivatives ---
		// dPc/dω = -skew(Pc)

		let dx_dwx = 0.0;
		let dx_dwy = z;
		let dx_dwz = -y;

		let dy_dwx = -z;
		let dy_dwy = 0.0;
		let dy_dwz = x;

		let dz_dwx = y;
		let dz_dwy = -x;
		let dz_dwz = 0.0;

		let du_dwx = fx * (dx_dwx * z - x * dz_dwx) * inv_z2;
		let du_dwy = fx * (dx_dwy * z - x * dz_dwy) * inv_z2;
		let du_dwz = fx * (dx_dwz * z - x * dz_dwz) * inv_z2;

		let dv_dwx = fy * (dy_dwx * z - y * dz_dwx) * inv_z2;
		let dv_dwy = fy * (dy_dwy * z - y * dz_dwy) * inv_z2;
		let dv_dwz = fy * (dy_dwz * z - y * dz_dwz) * inv_z2;

		// Fill Jacobian row pair
		let row_u = 2 * i;
		let row_v = 2 * i + 1;

		j[(row_u, 0)] = du_dwx;
		j[(row_u, 1)] = du_dwy;
		j[(row_u, 2)] = du_dwz;
		j[(row_u, 3)] = du_dtx;
		j[(row_u, 4)] = du_dty;
		j[(row_u, 5)] = du_dtz;

		j[(row_v, 0)] = dv_dwx;
		j[(row_v, 1)] = dv_dwy;
		j[(row_v, 2)] = dv_dwz;
		j[(row_v, 3)] = dv_dtx;
		j[(row_v, 4)] = dv_dty;
		j[(row_v, 5)] = dv_dtz;
	}

	let weight = 1.0 / (sigma_px * sigma_px);
	let mut jtj = weight * (j.transpose() * &j);

	if !jtj.try_inverse_mut() {
		let lambda = 1e-9;
		let out: Matrix<f64, Const<6>, Const<6>, ArrayStorage<f64, 6, 6>> =
			jtj + lambda * Matrix6::identity();
		jtj = out.try_inverse().unwrap();
	}

	jtj
}
