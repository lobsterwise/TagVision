use nalgebra::{matrix, Matrix3, Vector3};

use crate::cv::{
	geom::{PnPSolution, Pose3D, Pose3DWithError},
	solve::PnPSolver,
};

#[link(name = "IPPE", kind = "static", modifiers = "+whole-archive")]
extern "C" {
	pub fn ippe_solve_square(
		squareLength: f32,
		_imagePoints: *const f64,
		_cameraMatrix: *const f64,
		_distCoeffs: *const f64,
		_rvec1: *mut f64,
		_tvec1: *mut f64,
		reprojErr1: *mut f32,
		_rvec2: *mut f64,
		_tvec2: *mut f64,
		reprojErr2: *mut f32,
	);
}

pub struct IPPESolver {}

impl PnPSolver for IPPESolver {
	fn new() -> Self
	where
		Self: Sized,
	{
		Self {}
	}

	fn solve(
		&mut self,
		layout: &crate::cv::apriltag::layout::AprilTagLayout,
		detection: &crate::cv::apriltag::AprilTagDetection,
		intrinsics: &crate::cv::distort::OpenCVCameraIntrinsics,
	) -> Option<crate::cv::geom::PnPSolution> {
		let world_to_tag = layout.get_tag_pose(detection.id)?;

		// The image points are a interleaved 2-channel, so pass it in row-major.
		// The library expects start at top left, clockwise
		let image_points = [
			detection.corners.m14,
			detection.corners.m24,
			detection.corners.m13,
			detection.corners.m23,
			detection.corners.m12,
			detection.corners.m22,
			detection.corners.m11,
			detection.corners.m21,
		];
		let image_points = image_points.as_ptr();

		let (cam_matrix, dist) = intrinsics.to_matrices();

		let cam_matrix = cam_matrix.transpose().to_owned();
		let cam_matrix = cam_matrix.as_slice().as_ptr();

		let dist = dist.transpose().to_owned();
		let dist = dist.as_slice().as_ptr();

		let mut r1 = [0.0; 9];
		let mut t1 = [0.0; 3];
		let mut err1 = 0.0f32;
		let mut r2 = [0.0; 9];
		let mut t2 = [0.0; 3];
		let mut err2 = 0.0f32;

		unsafe {
			ippe_solve_square(
				layout.tag_size() as f32,
				image_points,
				cam_matrix,
				dist,
				r1.as_mut_ptr(),
				t1.as_mut_ptr(),
				(&mut err1) as *mut _,
				r2.as_mut_ptr(),
				t2.as_mut_ptr(),
				(&mut err2) as *mut _,
			);
		}

		let pose1 = Pose3DWithError {
			pose: Pose3D {
				t: Vector3::from_row_slice(&t1),
				r: Matrix3::from_row_slice(&r1),
			},
			error: err1 as f64,
		};
		let pose2 = Pose3DWithError {
			pose: Pose3D {
				t: Vector3::from_row_slice(&t2),
				r: Matrix3::from_row_slice(&r2),
			},
			error: err2 as f64,
		};

		let pose = choose_pose(pose1, pose2);
		let mut camera_to_tag = pose.pose;
		let error = pose.error;

		// Remap / rotate axes into correct frame
		let axis_remap = matrix![
			0.0, 0.0, 1.0;
			1.0, 0.0, 0.0;
			0.0, 1.0, 0.0
		];
		camera_to_tag.r = axis_remap * camera_to_tag.r * axis_remap.transpose();
		camera_to_tag.t = axis_remap * camera_to_tag.t;

		// The pose rotates in place but we want to swing around the camera.
		camera_to_tag.t = camera_to_tag.r.transpose() * camera_to_tag.t;

		let mut tag_to_camera = camera_to_tag.inverse();

		// Rotate to face the tag instead of away from it
		let face_tag = matrix![
			1.0, 0.0, 0.0;
			0.0, -1.0, 0.0;
			0.0, 0.0, -1.0
		];

		tag_to_camera.r = tag_to_camera.r * face_tag;

		// Compute relative to tag
		let world_to_camera = Pose3D {
			r: world_to_tag.r * tag_to_camera.r,
			t: world_to_tag.t + world_to_tag.r * tag_to_camera.t,
		};

		Some(PnPSolution::Multi(vec![Pose3DWithError {
			pose: world_to_camera,
			error,
		}]))
	}
}

fn choose_pose(p1: Pose3DWithError, p2: Pose3DWithError) -> Pose3DWithError {
	if p1.error < p2.error {
		p1
	} else {
		p2
	}
}
