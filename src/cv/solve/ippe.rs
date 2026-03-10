use nalgebra::{Matrix3, Point2, Point3, Vector3, SVD};

use crate::cv::{
	geom::{PnPSolution, Pose3D, Pose3DWithError},
	solve::PnPSolver,
};

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
		tag_width: f64,
	) -> Option<crate::cv::geom::PnPSolution> {
		let object_points = layout.get_tag_corners(detection.id, tag_width)?;
		let object_points = [
			Point3::new(object_points.m11, object_points.m21, object_points.m31),
			Point3::new(object_points.m12, object_points.m22, object_points.m32),
			Point3::new(object_points.m13, object_points.m23, object_points.m33),
			Point3::new(object_points.m14, object_points.m24, object_points.m34),
		];
		let image_points = [
			Point2::new(detection.corners.m11, detection.corners.m21),
			Point2::new(detection.corners.m12, detection.corners.m22),
			Point2::new(detection.corners.m13, detection.corners.m23),
			Point2::new(detection.corners.m14, detection.corners.m24),
		];

		let world_to_camera = solve_ippe(
			&object_points,
			&image_points,
			intrinsics.fx,
			intrinsics.fy,
			intrinsics.cx,
			intrinsics.cy,
		);

		dbg!(&world_to_camera);

		Some(PnPSolution::Multi(vec![Pose3DWithError {
			pose: world_to_camera,
			error: 0.0,
		}]))
	}
}

fn solve_ippe(
	object_points: &[Point3<f64>],
	image_points: &[Point2<f64>],
	fx: f64,
	fy: f64,
	cx: f64,
	cy: f64,
) -> Pose3D {
	assert!(object_points.len() == 4 && image_points.len() == 4);

	// --- Step 1: define local tag plane in 3D ---
	// Use object_points[0] as origin
	let origin = object_points[0];

	// Vector along edge 0->1 and 0->3 define local X and Y axes in tag plane
	let x_axis = Vector3::new(0.0, 1.0, 0.0); // Y
	let y_axis = Vector3::new(0.0, 0.0, 1.0); // Z
	let z_axis = x_axis.cross(&y_axis).normalize();

	// Ensure orthonormal frame
	let y_axis = z_axis.cross(&x_axis);

	let plane_to_world = Matrix3::from_columns(&[x_axis, y_axis, z_axis]);

	// --- Step 2: project object points to plane coordinates ---
	let mut plane_points = Vec::with_capacity(4);
	for p in object_points {
		let local = plane_to_world.transpose() * (p - origin);
		plane_points.push(Point2::new(local.x, local.y));
	}

	let normalized = normalize_points(image_points, fx, fy, cx, cy);

	// --- Step 4: compute homography ---
	let h = homography_dlt(&plane_points, &normalized);

	// --- Step 5: get the two IPPE poses ---
	let (p1, p2) = homography_to_pose(&h, &plane_to_world, origin, &z_axis);

	// --- Step 6: choose the pose with lowest reprojection error ---
	choose_pose(p1, p2, object_points, image_points)
}

fn choose_pose(p1: Pose3D, p2: Pose3D, obj: &[Point3<f64>], img: &[Point2<f64>]) -> Pose3D {
	let e1 = reprojection_error(&p1, obj, img);
	let e2 = reprojection_error(&p2, obj, img);

	if e1 < e2 {
		p1
	} else {
		p2
	}
}

fn normalize_points(pts: &[Point2<f64>], fx: f64, fy: f64, cx: f64, cy: f64) -> Vec<Point2<f64>> {
	pts.iter()
		.map(|p| Point2::new((p.x - cx) / fx, (p.y - cy) / fy))
		.collect()
}

fn homography_to_pose(
	h: &Matrix3<f64>,
	plane_to_world: &Matrix3<f64>,
	origin: Point3<f64>,
	z_axis: &Vector3<f64>,
) -> (Pose3D, Pose3D) {
	// Normalize homography
	let h = h / h[(2, 2)];
	let h1 = h.column(0);
	let h2 = h.column(1);
	let h3 = h.column(2);

	// Compute scale
	let scale = (h1.norm() + h2.norm()) / 2.0;

	// Rotation in plane coordinates
	let mut r1 = h1 / scale;
	let mut r2 = h2 / scale;
	r1 = r1.normalize();
	r2 = (r2 - r1 * r1.dot(&r2)).normalize();
	let r3 = r1.cross(&r2);

	let t_plane = h3 / scale;

	let r_plane = Matrix3::from_columns(&[r1, r2, r3]);

	// Convert back to world frame
	let r_world = plane_to_world * r_plane;
	let t_world = plane_to_world * t_plane + origin.coords;

	// First pose
	let pose1 = Pose3D {
		r: r_world,
		t: t_world,
	};

	// Second pose (IPPE reflection)
	let n = z_axis; // the normal of the tag plane
	let r_alt = r_world - 2.0 * (r_world * n) * n.transpose();
	let t_alt = t_world - 2.0 * (t_world.dot(&n)) * n;

	let pose2 = Pose3D { r: r_alt, t: t_alt };

	(pose1, pose2)
}

fn homography_dlt(obj: &[Point2<f64>], img: &[Point2<f64>]) -> Matrix3<f64> {
	let n = obj.len();

	let mut a = nalgebra::DMatrix::<f64>::zeros(2 * n, 9);

	for i in 0..n {
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

	let mut h = Matrix3::from_row_slice(&[h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]]);
	h /= h[(2, 2)];
	h
}

fn reprojection_error(pose: &Pose3D, obj: &[Point3<f64>], img: &[Point2<f64>]) -> f64 {
	let mut err = 0.0;

	for (p, obs) in obj.iter().zip(img.iter()) {
		let cam = pose.r * p.coords + pose.t;

		let u = cam.x / cam.z;
		let v = cam.y / cam.z;

		err += (u - obs.x).powi(2) + (v - obs.y).powi(2);
	}

	err
}
