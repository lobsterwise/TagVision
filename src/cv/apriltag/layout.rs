use std::collections::HashMap;

use nalgebra::{Matrix3x4, Vector3};
use serde::Deserialize;

use crate::cv::geom::Pose3D;

#[derive(Clone)]
pub struct AprilTagLayout {
	tags: HashMap<u8, Pose3D>,
	field: LayoutField,
	tag_size: f64,
}

#[derive(Deserialize, Copy, Clone)]
pub enum AprilTagLayoutPreset {
	#[serde(rename = "2024")]
	Layout2024,
	#[serde(rename = "2025_welded")]
	Layout2025Welded,
	#[serde(rename = "2026_andymark")]
	Layout2026Andymark,
	#[serde(rename = "2026_welded")]
	Layout2026Welded,
}

impl AprilTagLayoutPreset {
	fn get_layout(&self) -> &'static str {
		match self {
			Self::Layout2024 => include_str!("layouts/2024.json"),
			Self::Layout2025Welded => include_str!("layouts/2025-welded.json"),
			Self::Layout2026Andymark => include_str!("layouts/2026-andymark.json"),
			Self::Layout2026Welded => include_str!("layouts/2026-welded.json"),
		}
	}
}

impl AprilTagLayout {
	/// Loads the given WPILib AprilTag layout
	pub fn load(layout: AprilTagLayoutDeser, tag_size: f64) -> Self {
		let mut tags = HashMap::with_capacity(layout.tags.len());
		for tag in layout.tags {
			tags.insert(tag.id, Pose3D::from_tag_pose(tag.pose));
		}

		Self {
			tags,
			field: layout.field,
			tag_size,
		}
	}

	/// Loads one of the preset AprilTag layouts
	pub fn load_from_preset(layout: AprilTagLayoutPreset, tag_size: f64) -> Self {
		let data = layout.get_layout();

		let deser: AprilTagLayoutDeser =
			serde_json::from_str(data).expect("Preset layout is invalid");

		Self::load(deser, tag_size)
	}

	/// Gets the tag width in meters
	pub fn tag_size(&self) -> f64 {
		self.tag_size
	}

	/// Gets the 3D pose of a tag
	pub fn get_tag_pose(&self, tag: u8) -> Option<&Pose3D> {
		self.tags.get(&tag)
	}

	/// Gets the 3D corners of a tag
	pub fn get_tag_corners(&self, tag: u8) -> Option<Matrix3x4<f64>> {
		let center = self.get_tag_pose(tag)?;

		let corners = self.get_local_tag_corners();

		// Rotate to match the tag's plane
		let mut corners = center.r * corners;

		// Translate to match the tag's position
		for mut col in corners.column_iter_mut() {
			col += center.t;
		}

		Some(corners)
	}

	// Gets local tag corners on the YZ plane
	pub fn get_local_tag_corners(&self) -> Matrix3x4<f64> {
		let half_tag_size = self.tag_size / 2.0;

		// Corners in the tag's coordinate system, with the tag at the origin
		let c1 = Vector3::new(0.0, -half_tag_size, -half_tag_size);
		let c2 = Vector3::new(0.0, half_tag_size, -half_tag_size);
		let c3 = Vector3::new(0.0, half_tag_size, half_tag_size);
		let c4 = Vector3::new(0.0, -half_tag_size, half_tag_size);
		Matrix3x4::from_columns(&[c1, c2, c3, c4])
	}

	// Gets local tag corners on the XY plane
	pub fn get_local_tag_corners_xy(&self) -> Matrix3x4<f64> {
		let half_tag_size = self.tag_size / 2.0;

		// Corners in the tag's coordinate system, with the tag at the origin
		let c1 = Vector3::new(-half_tag_size, -half_tag_size, 0.0);
		let c2 = Vector3::new(half_tag_size, -half_tag_size, 0.0);
		let c3 = Vector3::new(half_tag_size, half_tag_size, 0.0);
		let c4 = Vector3::new(-half_tag_size, half_tag_size, 0.0);
		Matrix3x4::from_columns(&[c1, c2, c3, c4])
	}

	/// Checks whether a 2d translation is within some margin of the field borders
	pub fn within_margin(&self, x: f64, y: f64, margin: f64) -> bool {
		let x_within = x <= self.field.length + margin && x >= 0.0 - margin;
		let y_within = y <= self.field.width + margin && y >= 0.0 - margin;

		x_within && y_within
	}
}

/// A layout for positions of AprilTags on the field
#[derive(Deserialize)]
pub struct AprilTagLayoutDeser {
	pub tags: Vec<LayoutTag>,
	pub field: LayoutField,
}

#[derive(Deserialize, Clone)]
pub struct LayoutField {
	pub length: f64,
	pub width: f64,
}

/// A single tag in the layout
#[derive(Deserialize)]
pub struct LayoutTag {
	#[serde(alias = "ID")]
	pub id: u8,
	pub pose: LayoutPose,
}

#[derive(Deserialize)]
pub struct LayoutPose {
	pub translation: LayoutTranslation,
	pub rotation: LayoutRotation,
}

#[derive(Deserialize)]
pub struct LayoutTranslation {
	pub x: f64,
	pub y: f64,
	pub z: f64,
}

#[derive(Deserialize)]
pub struct LayoutRotation {
	pub quaternion: LayoutQuaternion,
}

#[derive(Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub struct LayoutQuaternion {
	pub w: f64,
	pub x: f64,
	pub y: f64,
	pub z: f64,
}

#[cfg(test)]
mod tests {
	use nalgebra::{matrix, Matrix3, Rotation3};

	use super::*;

	#[test]
	fn test_tag_corners_simple() {
		let tag_pose = Pose3D {
			t: matrix![0.0; 0.0; 0.0;],
			r: Matrix3::zeros(),
		};
		let expected = matrix![
			0.0, 0.0, 0.0, 0.0;
			1.0, -1.0, -1.0, 1.0;
			1.0, 1.0, -1.0, -1.0;
		];
		test_tag_corners(tag_pose, expected);
	}

	#[test]
	fn test_tag_corners_translation_only() {
		let tag_pose = Pose3D {
			t: matrix![5.0; 0.0; 2.0;],
			r: Matrix3::zeros(),
		};
		let expected = matrix![
			5.0, 5.0, 5.0, 5.0;
			1.0, -1.0, -1.0, 1.0;
			3.0, 3.0, 1.0, 1.0;
		];
		test_tag_corners(tag_pose, expected);
	}

	#[test]
	fn test_tag_corners_90deg_z() {
		let tag_pose = Pose3D {
			t: matrix![0.0; 0.0; 0.0;],
			r: Rotation3::from_euler_angles(0.0, 0.0, 90.0_f64.to_radians()).into_inner(),
		};
		let expected = matrix![
			-1.0, 1.0, 1.0, -1.0;
			0.0, 0.0, 0.0, 0.0;
			1.0, 1.0, -1.0, -1.0;
		];
		test_tag_corners(tag_pose, expected);
	}

	fn test_tag_corners(tag_pose: Pose3D, expected: Matrix3x4<f64>) {
		let mut tags = HashMap::new();
		tags.insert(0, tag_pose);

		let layout = AprilTagLayout {
			tags,
			field: LayoutField {
				length: 0.0,
				width: 0.0,
			},
			tag_size: 2.0,
		};

		let corners = layout.get_tag_corners(0).unwrap();
		if !corners.relative_eq(&expected, 0.01, 0.2) {
			dbg!(corners, expected);
			panic!("Did not match");
		}
	}
}
