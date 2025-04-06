use std::time::Instant;

use cv::apriltag::{params::AprilTagDetectorParams, AprilTagDetector};
use image::GrayImage;
use nokhwa::{
	pixel_format::LumaFormat,
	utils::{CameraIndex, RequestedFormat, RequestedFormatType},
	Camera,
};

mod config;
mod cv;

fn main() {
	let params = AprilTagDetectorParams {
		thread_count: 1,
		quad_decimate: 3.0,
		..Default::default()
	};
	let detector = AprilTagDetector::new(params);
	detector.debug();
	println!("Detector Created");
	// let image = GrayImage::new(1600, 1200);

	// let mut cam = Camera::new(
	// 	CameraIndex::Index(0),
	// 	RequestedFormat::new::<LumaFormat>(RequestedFormatType::AbsoluteHighestFrameRate),
	// )
	// .unwrap();

	// loop {
	// 	let frame = cam.frame();
	// 	if let Ok(frame) = frame {
	// 		let frame = frame.decode_image::<LumaFormat>();
	// 		if let Ok(frame) = frame {
	// 		}
	// 	}
	// }

	let image = image::open("apriltags2.jpg").unwrap();
	// let image = GrayImage::new(1600, 1200);
	let image: image::ImageBuffer<image::Luma<u8>, Vec<u8>> = image.into();
	println!("Image loaded");

	let detections = detector.detect_markers(image.clone());
	println!("Detections made");
	detector.debug();
	dbg!(&detections);
	for detection in detections.iter() {
		println!("Detection {}", detection.id);
	}

	let count = 100;
	let mut sum = 0.0;
	for _ in 0..count {
		let image = image.clone();
		let start = Instant::now();
		detector.detect_markers(image);
		let time = Instant::now() - start;
		sum += time.as_secs_f32();
	}
	println!("{}ms average", sum / count as f32 * 1000.0);
	println!("{}fps average", 1.0 / (sum / count as f32));

	println!("Hello, world!");
}
