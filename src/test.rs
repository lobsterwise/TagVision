use std::sync::Arc;

use crate::cv::apriltag::{params::AprilTagDetectorParams, AprilTagDetections, AprilTagDetector};

// 3 tags with one of them slightly covered
#[tokio::test]
async fn test_3_apriltags() {
	let detections = get_image_detections(include_bytes!("../test_images/3.jpg")).await;
	assert!(detections.size() >= 2);
}

// 5 tags
#[tokio::test]
async fn test_5_apriltags() {
	let detections = get_image_detections(include_bytes!("../test_images/5.jpg")).await;
	println!("{}", detections.get_detection(0).unwrap().area());
	assert_eq!(detections.size(), 5);
}

async fn get_image_detections(image: &[u8]) -> AprilTagDetections {
	let detector = AprilTagDetector::new(AprilTagDetectorParams {
		thread_count: 1,
		quad_decimate: 3.0,
		time_profile: false,
	});

	let image = image::load_from_memory(image).unwrap().into_luma8();
	let image = Arc::new(image);

	detector.detect_markers(&image).await
}
