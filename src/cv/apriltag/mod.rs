mod ffi;
pub mod layout;
pub mod params;

use ffi::*;
use imageproc::pixelops::interpolate;
use std::{fmt::Debug, sync::Arc};

use image::{GrayImage, Rgb, RgbImage};
use nalgebra::{Matrix2x4, Matrix3x4, Matrix4x2, Vector2};
use params::AprilTagDetectorParams;

use super::distort::OpenCVCameraIntrinsics;

/// Wrapper around the AprilTag library for detecting AprilTags
pub struct AprilTagDetector {
	inner: Arc<AprilTagDetectorInner>,
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

		Self { inner }
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
			center: Vector2::from_row_slice(&raw_detection.c),
			corners: Matrix2x4::from_row_slice(raw_detection.p.as_flattened()).transpose(),
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
	pub center: Vector2<f64>,
	pub corners: Matrix4x2<f64>,
}

impl AprilTagDetection {
	/// Draws this tag detection on an image
	pub fn draw(&self, image: &mut RgbImage) {
		imageproc::drawing::draw_antialiased_line_segment_mut(
			image,
			(self.corners[(0, 0)] as i32, self.corners[(0, 1)] as i32),
			(self.corners[(1, 0)] as i32, self.corners[(1, 1)] as i32),
			Rgb([255, 0, 0]),
			interpolate,
		);
		imageproc::drawing::draw_antialiased_line_segment_mut(
			image,
			(self.corners[(1, 0)] as i32, self.corners[(1, 1)] as i32),
			(self.corners[(2, 0)] as i32, self.corners[(2, 1)] as i32),
			Rgb([255, 0, 0]),
			interpolate,
		);
		imageproc::drawing::draw_antialiased_line_segment_mut(
			image,
			(self.corners[(2, 0)] as i32, self.corners[(2, 1)] as i32),
			(self.corners[(3, 0)] as i32, self.corners[(3, 1)] as i32),
			Rgb([255, 0, 0]),
			interpolate,
		);
		imageproc::drawing::draw_antialiased_line_segment_mut(
			image,
			(self.corners[(3, 0)] as i32, self.corners[(3, 1)] as i32),
			(self.corners[(0, 0)] as i32, self.corners[(0, 1)] as i32),
			Rgb([255, 0, 0]),
			interpolate,
		);
	}

	/// Undistorts this detection's corners using the given camera intrinsics
	pub fn get_undistorted_corners(&self, intrinsics: &OpenCVCameraIntrinsics) -> Matrix3x4<f64> {
		let corners = self.corners.transpose();

		// Undistort each corner
		let v1 = intrinsics.unproject_one(&corners.column(0).clone_owned());
		let v2 = intrinsics.unproject_one(&corners.column(1).clone_owned());
		let v3 = intrinsics.unproject_one(&corners.column(2).clone_owned());
		let v4 = intrinsics.unproject_one(&corners.column(3).clone_owned());

		Matrix3x4::from_columns(&[v1, v2, v3, v4])
	}
}
