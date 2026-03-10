mod ffi;
pub mod layout;
pub mod params;

use ffi::*;
use imageproc::pixelops::interpolate;
use std::{fmt::Debug, sync::Arc};

use image::{GrayImage, Rgb, RgbImage};
use nalgebra::{matrix, Matrix2x4, Matrix3, Matrix3x4, Matrix4x2, Vector2};
use params::AprilTagDetectorParams;

use crate::{config::TagFilters, cv::geom::Pose3DWithError};

use super::distort::OpenCVCameraIntrinsics;

/// Wrapper around the AprilTag library for detecting AprilTags
pub struct AprilTagDetector {
	inner: Arc<AprilTagDetectorInner>,
	time_profile: bool,
}

struct AprilTagDetectorInner {
	inner: *mut _AprilTagDetector,
	family: *mut _AprilTagFamily,
}

unsafe impl Send for AprilTagDetectorInner {}
unsafe impl Sync for AprilTagDetectorInner {}

impl AprilTagDetector {
	pub fn new(params: AprilTagDetectorParams) -> Self {
		let (inner, family) = unsafe {
			let inner = apriltag_detector_create();

			// Add the 36h11 family
			let family = tag36h11_create();
			apriltag_detector_add_family_bits(inner, family, 2);

			// Add configuration
			(*inner).nthreads = params.thread_count as i32;
			(*inner).quad_decimate = params.quad_decimate;

			(inner, family)
		};

		let inner = Arc::new(AprilTagDetectorInner { inner, family });

		Self {
			inner,
			time_profile: params.time_profile,
		}
	}

	/// Detect markers in an image
	pub async fn detect_markers(&self, image: &Arc<GrayImage>) -> AprilTagDetections {
		let width = image.width() as i32;
		let height = image.height() as i32;

		let inner = self.inner.clone();
		let image = image.clone();

		let result = tokio::task::spawn_blocking(move || {
			let image = _ImageU8 {
				width,
				height,
				stride: width,
				buf: image.as_ptr() as *mut u8,
			};
			let detections = unsafe { apriltag_detector_detect(inner.inner, &image) };
			AprilTagDetections { detections }
		})
		.await;

		if self.time_profile {
			unsafe {
				(*(*self.inner.inner).tp).report();
			}
		}

		match result {
			Ok(result) => result,
			Err(..) => AprilTagDetections {
				detections: std::ptr::null_mut(),
			},
		}
	}
}

impl Drop for AprilTagDetector {
	fn drop(&mut self) {
		unsafe {
			apriltag_detector_destroy(self.inner.inner);
			tag36h11_destroy(self.inner.family);
		}
	}
}

impl Debug for AprilTagDetector {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		unsafe { write!(f, "{:?}", *self.inner.inner) }
	}
}

/// Wrapper struct representing the result from tag detection
pub struct AprilTagDetections {
	detections: *mut _ZArray,
}

unsafe impl Send for AprilTagDetections {}

impl AprilTagDetections {
	/// Gets the detection at the given index
	pub fn get_detection(&self, idx: usize) -> Option<AprilTagDetection> {
		if self.detections.is_null() {
			return None;
		}

		let raw_detection = unsafe { (*self.detections).get::<*const _AprilTagDetection>(idx) };
		let Some(raw_detection) = raw_detection else {
			return None;
		};

		let raw_detection = unsafe { raw_detection.as_ref() };
		let Some(raw_detection) = raw_detection else {
			return None;
		};

		Some(AprilTagDetection {
			id: raw_detection.id as u8,
			hamming: raw_detection.hamming as u8,
			decision_margin: raw_detection.decision_margin,
			homography: Matrix3::from_row_slice(unsafe {
				std::slice::from_raw_parts((*raw_detection.h).data, 3 * 3)
			}),
			center: Vector2::from_row_slice(&raw_detection.c),
			corners: Matrix4x2::from_row_slice(raw_detection.p.as_flattened()).transpose(),
		})
	}

	/// Iterates over the detections
	pub fn iter<'a>(&'a self) -> impl Iterator<Item = AprilTagDetection> + 'a {
		(0..self.size()).map(|x| unsafe { self.get_detection(x).unwrap_unchecked() })
	}

	/// Gets the size of the detections
	pub fn size(&self) -> usize {
		unsafe { (*self.detections).size as usize }
	}

	/// Gets these detections as a vector
	pub fn to_vec(self) -> Vec<AprilTagDetection> {
		self.iter().collect()
	}
}

impl Drop for AprilTagDetections {
	fn drop(&mut self) {
		unsafe { apriltag_detections_destroy(self.detections) }
	}
}

impl Debug for AprilTagDetections {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		if self.detections.is_null() {
			write!(f, "Null")
		} else {
			unsafe {
				dbg!(&*self.detections);
			}
			for detection in self.iter() {
				writeln!(f, "{:?}", detection)?;
			}

			Ok(())
		}
	}
}

/// Wrapper struct for a single AprilTag detection
#[derive(Debug)]
pub struct AprilTagDetection {
	pub id: u8,
	pub hamming: u8,
	pub decision_margin: f32,
	pub homography: Matrix3<f64>,
	pub center: Vector2<f64>,
	// Apriltag corners, with the bottom left being corner 0 and going counterclockwise from there
	pub corners: Matrix2x4<f64>,
}

impl AprilTagDetection {
	/// Solves for pose using homography on this detection, returning transform of camera relative to tag
	pub fn solve(&self, intrinsics: &OpenCVCameraIntrinsics, tag_width: f64) -> Pose3DWithError {
		let mut t1 = [0.0; 3];
		let mut t1 = _MatD {
			nrows: 3,
			ncols: 1,
			data: t1.as_mut_ptr(),
		};
		let mut r1 = [0.0; 3 * 3];
		let mut r1 = _MatD {
			nrows: 3,
			ncols: 3,
			data: r1.as_mut_ptr(),
		};
		let mut pose1 = _AprilTagPose {
			t: (&mut t1) as *mut _,
			r: (&mut r1) as *mut _,
		};
		let mut t2 = [0.0; 3];
		let mut t2 = _MatD {
			nrows: 3,
			ncols: 1,
			data: t2.as_mut_ptr(),
		};
		let mut r2 = [0.0; 3 * 3];
		let mut r2 = _MatD {
			nrows: 3,
			ncols: 3,
			data: r2.as_mut_ptr(),
		};
		let mut pose2 = _AprilTagPose {
			t: (&mut t2) as *mut _,
			r: (&mut r2) as *mut _,
		};

		let mut err1 = 0.0;
		let mut err2 = 0.0;

		let mut h = _MatD {
			nrows: 3,
			ncols: 3,
			// SAFETY: Estimation does not modify homography of detection
			data: (self.homography.as_slice().as_ptr()) as *mut _,
		};

		let corners = [
			[self.corners.m11, self.corners.m21],
			[self.corners.m12, self.corners.m22],
			[self.corners.m13, self.corners.m23],
			[self.corners.m14, self.corners.m24],
		];

		let det = _AprilTagDetection {
			// SAFETY: Estimation does not access the family
			family: std::ptr::null_mut(),
			id: self.id as i32,
			hamming: self.hamming as i32,
			decision_margin: self.decision_margin,
			h: (&mut h) as *mut _,
			c: self.center.as_slice().try_into().unwrap(),
			p: corners,
		};

		let mut info = _AprilTagDetectionInfo {
			det: (&det) as *const _,
			tagsize: tag_width,
			fx: intrinsics.fx,
			fy: intrinsics.fy,
			cx: intrinsics.cx,
			cy: intrinsics.cy,
		};

		unsafe {
			estimate_tag_pose_orthogonal_iteration(
				(&mut info) as *mut _,
				(&mut err1) as *mut _,
				(&mut pose1) as *mut _,
				(&mut err2) as *mut _,
				(&mut pose2) as *mut _,
				50,
			)
		};

		let pose1 = unsafe { pose1.to_pose3d() };
		let pose2 = unsafe { pose2.to_pose3d_optional() };

		// Pick whichever pose is in front of the tag

		// dbg!(&pose1.t.z);
		// if let Some(p2) = &pose2 {
		// 	dbg!(&p2.t.z);
		// }
		let (mut pose, error) = if pose1.t.z > 0.0 {
			(pose1, err1)
		} else if let Some(pose2) = pose2 {
			if pose2.t.z > 0.0 {
				(pose2, err2)
			} else {
				// Pick based on reproj error
				if err1 < err2 {
					(pose1, err1)
				} else {
					(pose2, err2)
				}
			}
		} else {
			(pose1, err1)
		};

		// Fix if both solutions are behind the tag
		if pose.t.z < 0.0 {
			// Rotate 180 degrees around z
			let r_matrix = matrix![
				-1.0, 0.0, 0.0;
				0.0, -1.0, 0.0;
				0.0, 0.0, 1.0;
			];

			pose.t *= -1.0;
			pose.r = r_matrix * pose.r;
		}

		Pose3DWithError { pose, error }
	}

	/// Gets the perimeter of this tag in pixels
	pub fn perimeter(&self) -> f64 {
		let side0 = (self.corners.column(1) - self.corners.column(0)).norm();
		let side1 = (self.corners.column(2) - self.corners.column(1)).norm();
		let side2 = (self.corners.column(3) - self.corners.column(2)).norm();
		let side3 = (self.corners.column(0) - self.corners.column(3)).norm();

		side0 + side1 + side2 + side3
	}

	/// Gets the area of this tag in pixels
	pub fn area(&self) -> f64 {
		// Get the determinant of the matrix of the vectors from the bottom left corner
		let bottom_left = self.corners.column(0);
		let bottom_right = self.corners.column(1);
		let top_left = self.corners.column(3);

		let matrix = matrix![
			top_left.x - bottom_left.x, bottom_right.x - bottom_left.x;
			top_left.y - bottom_left.y, bottom_right.y - bottom_left.y;
		];

		matrix.determinant().abs()
	}

	/// Gets whether this tag is filtered out
	pub fn is_filtered(&self, filters: &TagFilters) -> bool {
		let area = self.area();
		if let Some(min_area) = filters.min_area {
			if area < min_area {
				return false;
			}
		}
		if let Some(max_area) = filters.max_area {
			if area > max_area {
				return false;
			}
		}

		let perimeter = self.perimeter();
		if let Some(min_perimeter) = filters.min_perimeter {
			if perimeter < min_perimeter {
				return false;
			}
		}
		if let Some(max_perimeter) = filters.max_perimeter {
			if perimeter > max_perimeter {
				return false;
			}
		}

		false
	}

	/// Draws this tag detection on an image
	pub fn draw(&self, image: &mut RgbImage) {
		imageproc::drawing::draw_antialiased_line_segment_mut(
			image,
			(self.corners[(0, 0)] as i32, self.corners[(1, 0)] as i32),
			(self.corners[(0, 1)] as i32, self.corners[(1, 1)] as i32),
			Rgb([255, 0, 0]),
			interpolate,
		);
		imageproc::drawing::draw_antialiased_line_segment_mut(
			image,
			(self.corners[(0, 1)] as i32, self.corners[(1, 1)] as i32),
			(self.corners[(0, 2)] as i32, self.corners[(1, 2)] as i32),
			Rgb([255, 0, 0]),
			interpolate,
		);
		imageproc::drawing::draw_antialiased_line_segment_mut(
			image,
			(self.corners[(0, 2)] as i32, self.corners[(1, 2)] as i32),
			(self.corners[(0, 3)] as i32, self.corners[(1, 3)] as i32),
			Rgb([255, 0, 0]),
			interpolate,
		);
		imageproc::drawing::draw_antialiased_line_segment_mut(
			image,
			(self.corners[(0, 3)] as i32, self.corners[(1, 3)] as i32),
			(self.corners[(0, 0)] as i32, self.corners[(1, 0)] as i32),
			Rgb([255, 0, 0]),
			interpolate,
		);

		// First corner dot
		imageproc::drawing::draw_filled_circle_mut(
			image,
			(self.corners[(0, 0)] as i32, self.corners[(1, 0)] as i32),
			2,
			Rgb([0, 255, 0]),
		);
	}

	/// Undistorts this detection's corners using the given camera intrinsics, returning normalized rays
	pub fn get_undistorted_corners_rays(
		&self,
		intrinsics: &OpenCVCameraIntrinsics,
	) -> Matrix3x4<f64> {
		let v1 = intrinsics
			.unproject_one(&self.corners.column(0).clone_owned())
			.normalize();
		let v2 = intrinsics
			.unproject_one(&self.corners.column(1).clone_owned())
			.normalize();
		let v3 = intrinsics
			.unproject_one(&self.corners.column(2).clone_owned())
			.normalize();
		let v4 = intrinsics
			.unproject_one(&self.corners.column(3).clone_owned())
			.normalize();

		Matrix3x4::from_columns(&[v1, v2, v3, v4])
	}

	/// Undistorts this detection's corners using the given camera intrinsics, returning new pixel coordinates
	pub fn get_undistorted_corners_pixels(
		&self,
		intrinsics: &OpenCVCameraIntrinsics,
	) -> Matrix2x4<f64> {
		let v1 = intrinsics.unproject_one(&self.corners.column(0).clone_owned());
		let v2 = intrinsics.unproject_one(&self.corners.column(1).clone_owned());
		let v3 = intrinsics.unproject_one(&self.corners.column(2).clone_owned());
		let v4 = intrinsics.unproject_one(&self.corners.column(3).clone_owned());

		Matrix2x4::from_columns(&[
			intrinsics.to_pixel_coordinates(v1),
			intrinsics.to_pixel_coordinates(v2),
			intrinsics.to_pixel_coordinates(v3),
			intrinsics.to_pixel_coordinates(v4),
		])
	}
}
