use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, Point2, Point3, Vector2, Vector3, SVD};

use crate::{
	cam,
	cv::{
		geom::{PnPSolution, Pose3D, Pose3DWithError},
		solve::PnPSolver,
	},
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
		// let object_points = layout.get_local_tag_corners_xy();
		// let object_points = [
		// 	Point3::new(object_points.m11, object_points.m21, object_points.m31),
		// 	Point3::new(object_points.m12, object_points.m22, object_points.m32),
		// 	Point3::new(object_points.m13, object_points.m23, object_points.m33),
		// 	Point3::new(object_points.m14, object_points.m24, object_points.m34),
		// ];
		// let image_points = [
		// 	Point2::new(detection.corners.m11, detection.corners.m21),
		// 	Point2::new(detection.corners.m12, detection.corners.m22),
		// 	Point2::new(detection.corners.m13, detection.corners.m23),
		// 	Point2::new(detection.corners.m14, detection.corners.m24),
		// ];
		dbg!(&detection.corners);
		let image_points = detection.corners.transpose().to_owned();
		let image_points = image_points.as_slice().as_ptr();
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

		dbg!(&pose1, &pose2);

		// dbg!(&object_points, &image_points);

		// let (pose1, pose2) = solve_ippe_square(
		// 	&object_points,
		// 	&image_points,
		// 	intrinsics.fx,
		// 	intrinsics.fy,
		// 	intrinsics.cx,
		// 	intrinsics.cy,
		// );

		let pose = choose_pose(pose1, pose2);

		Some(PnPSolution::Multi(vec![pose]))
	}
}

fn solve_ippe_square(
	object_points: &[Point3<f64>; 4], // must be square, order: TL, TR, BR, BL
	image_points: &[Point2<f64>; 4],
	fx: f64,
	fy: f64,
	cx: f64,
	cy: f64,
) -> (Pose3DWithError, Pose3DWithError) {
	// Step 1: normalize image points
	let img_norm: [Vector2<f64>; 4] = image_points
		.iter()
		.map(|p| Vector2::new((p.x - cx) / fx, (p.y - cy) / fy))
		.collect::<Vec<_>>()
		.try_into()
		.unwrap();

	// Step 2: canonical square in XY plane
	let s = 1.0;
	let canonical = [
		Vector2::new(-s / 2.0, -s / 2.0),
		Vector2::new(s / 2.0, -s / 2.0),
		Vector2::new(s / 2.0, s / 2.0),
		Vector2::new(-s / 2.0, s / 2.0),
	];

	// Step 3: homography DLT: canonical -> normalized image points
	let h = homography_dlt(&canonical, &img_norm);

	// Step 4: IPPE rotation computation
	let (ra, rb) = compute_ippe_rotations(&h);

	// Step 5: compute translation for both rotations
	let ta = compute_ippe_translation(&canonical, &img_norm, &ra);
	let tb = compute_ippe_translation(&canonical, &img_norm, &rb);

	// Step 6: wrap poses with reprojection error
	let pose1 = Pose3DWithError {
		pose: Pose3D { r: ra, t: ta },
		error: reprojection_error(&Pose3D { r: ra, t: ta }, object_points, &img_norm),
	};
	let pose2 = Pose3DWithError {
		pose: Pose3D { r: rb, t: tb },
		error: reprojection_error(&Pose3D { r: rb, t: tb }, object_points, &img_norm),
	};

	(pose1, pose2)
}

fn compute_ippe_rotations(h: &Matrix3<f64>) -> (Matrix3<f64>, Matrix3<f64>) {
	let r1 = Matrix3::from_columns(&[
		h.column(0).xyz().normalize(),
		h.column(1).xyz().normalize(),
		h.column(0).cross(&h.column(1)).xyz().normalize(),
	]);

	// mirror solution
	let r2 = Matrix3::new(
		r1[(0, 0)],
		-r1[(0, 1)],
		-r1[(0, 2)],
		r1[(1, 0)],
		-r1[(1, 1)],
		-r1[(1, 2)],
		r1[(2, 0)],
		-r1[(2, 1)],
		-r1[(2, 2)],
	);

	(r1, r2)
}
// Analytic translation for square tags
fn compute_ippe_translation(
	canonical: &[Vector2<f64>; 4],
	img: &[Vector2<f64>; 4],
	r: &Matrix3<f64>,
) -> Vector3<f64> {
	let mut t = Vector3::zeros();
	for i in 0..4 {
		let p = Vector3::new(canonical[i].x, canonical[i].y, 0.0);
		let cam = Vector3::new(img[i].x, img[i].y, 1.0) - r * p;
		t += cam;
	}
	t / 4.0
}

fn homography_dlt(obj: &[Vector2<f64>], img: &[Vector2<f64>]) -> Matrix3<f64> {
	let mut a = nalgebra::DMatrix::<f64>::zeros(8, 9);

	for i in 0..4 {
		let x = obj[i].x;
		let y = obj[i].y;
		let u = img[i].x;
		let v = img[i].y;

		a[(2 * i, 0)] = -x;
		a[(2 * i, 1)] = -y;
		a[(2 * i, 2)] = -1.0;
		a[(2 * i, 6)] = u * x;
		a[(2 * i, 7)] = u * y;
		a[(2 * i, 8)] = u;

		a[(2 * i + 1, 3)] = -x;
		a[(2 * i + 1, 4)] = -y;
		a[(2 * i + 1, 5)] = -1.0;
		a[(2 * i + 1, 6)] = v * x;
		a[(2 * i + 1, 7)] = v * y;
		a[(2 * i + 1, 8)] = v;
	}

	let svd = SVD::new(a, true, true);
	let v_t = svd.v_t.unwrap();
	let h = v_t.row(v_t.nrows() - 1);
	let mut h3 = Matrix3::from_row_slice(&[h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]]);
	h3 /= h3[(2, 2)];
	h3
}

/// Reprojection error (squared)
fn reprojection_error(pose: &Pose3D, obj: &[Point3<f64>; 4], img: &[Vector2<f64>; 4]) -> f64 {
	obj.iter()
		.zip(img.iter())
		.map(|(p, obs)| {
			let cam = pose.r * p.coords + pose.t;
			let u = cam.x / cam.z;
			let v = cam.y / cam.z;
			(u - obs.x).powi(2) + (v - obs.y).powi(2)
		})
		.sum()
}

fn choose_pose(p1: Pose3DWithError, p2: Pose3DWithError) -> Pose3DWithError {
	if p1.error < p2.error {
		p1
	} else {
		p2
	}
}
