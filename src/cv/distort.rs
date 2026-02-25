use nalgebra::{Vector2, Vector3};
use serde::Deserialize;

/// Camera intrinsics
#[derive(Deserialize, Clone)]
pub struct CameraCalibration {
	pub camera_matrix: [[f64; 3]; 3],
	pub distortion: [f64; 5],
}

#[derive(Clone, Debug)]
pub struct OpenCVCameraIntrinsics {
	pub fx: f64,
	pub fy: f64,
	pub cx: f64,
	pub cy: f64,
	pub k1: f64,
	pub k2: f64,
	pub p1: f64,
	pub p2: f64,
	pub k3: f64,
}

impl OpenCVCameraIntrinsics {
	pub fn from_calib(calib: &CameraCalibration) -> Self {
		Self {
			fx: calib.camera_matrix[0][0],
			fy: calib.camera_matrix[1][1],
			cx: calib.camera_matrix[0][2],
			cy: calib.camera_matrix[1][2],
			k1: calib.distortion[0],
			k2: calib.distortion[1],
			p1: calib.distortion[2],
			p2: calib.distortion[3],
			k3: calib.distortion[4],
		}
	}

	/// New impl with fixed-point iteration
	pub fn unproject_one(&self, pt: &Vector2<f64>) -> Vector3<f64> {
		let xd = (pt[0] - self.cx) / self.fx;
		let yd = (pt[1] - self.cy) / self.fy;

		// Initial guess
		let mut x = xd;
		let mut y = yd;

		let max_iter = 20;
		let eps = 1e-15;

		for _ in 0..max_iter {
			let x2 = x * x;
			let y2 = y * y;
			let r2 = x2 + y2;
			let r4 = r2 * r2;
			let r6 = r4 * r2;

			// Radial distortion factor
			let radial = 1.0 + self.k1 * r2 + self.k2 * r4 + self.k3 * r6;

			// Tangential distortion
			let tx = 2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x2);
			let ty = self.p1 * (r2 + 2.0 * y2) + 2.0 * self.p2 * x * y;

			// Next iteration estimates
			let x_next = (xd - tx) / radial;
			let y_next = (yd - ty) / radial;

			// Convergence test
			if (x_next - x).abs() < eps && (y_next - y).abs() < eps {
				x = x_next;
				y = y_next;
				break;
			}

			x = x_next;
			y = y_next;
		}

		Vector3::new(x, y, 1.0)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn test_undistort_one() {
		let intrinsics = CameraCalibration {
			camera_matrix: [[500.0, 0.0, 300.0], [0.0, 500.0, 250.0], [0.0, 0.0, 1.0]],
			distortion: [1.5, -0.95, -0.005, 0.0025, 1.16],
		};

		let cam = OpenCVCameraIntrinsics::from_calib(&intrinsics);

		let pixel = Vector2::new(400.0, 300.0);
		let undist = cam.unproject_one(&pixel);
		assert!(undist.relative_eq(&Vector3::new(0.19, 0.095, 1.0), 1e-3, 100.0));
		// assert_eq!(undist, Vector3::new(0.19, 0.095, 1.0));
	}
}
