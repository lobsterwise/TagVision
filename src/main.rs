use std::{fs::File, time::Instant};

use config::Config;
use cv::apriltag::{params::AprilTagDetectorParams, AprilTagDetector};
use runtime::Runtime;

/// Camera hardware interfaces
mod cam;
/// Program configuration
mod config;
/// Computer vision, such as tag detection and pose estimation
mod cv;
/// Individual vision camera modules doing their own processing
mod module;
/// Output to NetworkTables and CameraServer
mod output;
/// Runtime for all of the modules, handling camera capture, processing, and output
mod runtime;
/// General utilities
mod util;

#[tokio::main(flavor = "multi_thread")]
async fn main() {
	let config_file = File::open("config.json").expect("Failed to open config file");
	let config: Config = serde_json::from_reader(config_file).expect("Failed to parse config file");

	let runtime = Runtime::new(config);
	println!("Runtime initialized");

	runtime.run_forever().await;
}

fn debug() {
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

	let detections = detector.detect_markers(&image);
	println!("Detections made");
	detector.debug();
	dbg!(&detections);
	for detection in detections.iter() {
		println!("Detection {}", detection.id);
	}

	let count = 100;
	let mut sum = 0.0;
	for _ in 0..count {
		let start = Instant::now();
		detector.detect_markers(&image);
		let time = Instant::now() - start;
		sum += time.as_secs_f32();
	}
	println!("{}ms average", sum / count as f32 * 1000.0);
	println!("{}fps average", 1.0 / (sum / count as f32));
}
