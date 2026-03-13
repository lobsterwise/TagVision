use nalgebra::matrix;

use crate::cv::{
	apriltag::{layout::AprilTagLayout, AprilTagDetection},
	distort::OpenCVCameraIntrinsics,
	geom::{PnPSolution, Pose3D, Pose3DWithError},
};

use super::PnPSolver;

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
