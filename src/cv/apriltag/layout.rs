use std::collections::HashMap;

use nalgebra::{Matrix3x4, Vector3};
use serde::Deserialize;

use crate::cv::geom::Pose3D;

#[derive(Clone)]
pub struct AprilTagLayout {
	field: LayoutField,
	tags: HashMap<u8, Pose3D>,
}

#[derive(Deserialize, Copy, Clone)]
pub enum AprilTagLayoutPreset {
	#[serde(rename = "2024")]
	Layout2024,
	#[serde(rename = "2025_welded")]
	Layout2025Welded,
}

impl AprilTagLayout {
	/// Loads the given WPILib AprilTag layout
	pub fn load(layout: AprilTagLayoutDeser) -> Self {
		let mut tags = HashMap::with_capacity(layout.tags.len());
		for tag in layout.tags {
			tags.insert(tag.id, Pose3D::from_tag_pose(tag.pose));
		}

		Self {
			field: layout.field,
			tags,
		}
	}

	/// Loads one of the preset AprilTag layouts
	pub fn load_from_preset(layout: AprilTagLayoutPreset) -> Self {
		let data = match layout {
			AprilTagLayoutPreset::Layout2024 => include_str!("layouts/2024.json"),
			AprilTagLayoutPreset::Layout2025Welded => include_str!("layouts/2025-welded.json"),
		};

		let deser = serde_json::from_str(data).expect("Preset layout is invalid");
		Self::load(deser)
	}

	/// Gets the 3D pose of a tag
	pub fn get_tag_pose(&self, tag: u8) -> Option<&Pose3D> {
		self.tags.get(&tag)
	}

	/// Gets the 3D corners of a tag
	pub fn get_tag_corners(&self, tag: u8, tag_size: f64) -> Option<Matrix3x4<f64>> {
		let center = self.get_tag_pose(tag)?;
		
		// We have to rotate by 90 degrees as the conventions of the layout is to have 0deg mean a sideways facing tag.
		// We are trying to get the coordinate system of the tag's plane.
		let mut modified_center = center.clone();
		modified_center.ry += 90.0_f64.to_radians();
		let tag_plane = modified_center.get_rotation_matrix();
		
		let half_tag_size = tag_size / 2.0;

		// Corners in the tag's coordinate system, with the tag at the origin
		let c1 = Vector3::new(half_tag_size, half_tag_size, 0.0);
		let c2 = Vector3::new(-half_tag_size, half_tag_size, 0.0);
		let c3 = Vector3::new(-half_tag_size, -half_tag_size, 0.0);
		let c4 = Vector3::new(half_tag_size, -half_tag_size, 0.0);
		let corners = Matrix3x4::from_columns(&[c1, c2, c3, c4]);

		// Rotate to match the tag's plane
		let mut corners = tag_plane * corners;

		// Translate to match the tag's position
		let translation = Vector3::new(center.x, center.y, center.z);
		for mut col in corners.column_iter_mut() {
			col += translation;
		}

		Some(corners)
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
	#[serde(rename = "ID")]
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
