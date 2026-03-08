use std::{path::PathBuf, process::Command};

use crate::{cam::CameraSetupError, config::CameraConfig, sys::readlink};

/// Looks up a v4l camera ID (from /dev/v4l/by-id) to find it's index
pub fn lookup_camera_id_linux(camera_id: &str) -> std::io::Result<u32> {
	let path = PathBuf::from("/dev/v4l/by-path/").join(format!("{camera_id}-video-index0"));
	if !path.exists() {
		return Err(std::io::Error::new(
			std::io::ErrorKind::NotFound,
			"Camera file not found".to_string(),
		));
	}
	let result_path = readlink(&path)?;
	let Some(filename) = result_path.file_name() else {
		return Err(std::io::Error::new(
			std::io::ErrorKind::IsADirectory,
			"Filename not present".to_string(),
		));
	};

	let Some(last_char) = filename.to_string_lossy().chars().last() else {
		return Err(std::io::Error::new(
			std::io::ErrorKind::InvalidFilename,
			"Index not found in filename".to_string(),
		));
	};

	String::from(last_char).parse().map_err(|_| {
		std::io::Error::new(
			std::io::ErrorKind::InvalidFilename,
			"Failed to parse camera index".to_string(),
		)
	})
}

/// Configures a camera property using v4l2-ctl
pub fn v4l2_conf(cam_index: u8, property: &str, value: &str) -> Result<(), CameraSetupError> {
	let mut cmd = Command::new("v4l2-ctl");
	cmd.arg("-d");
	cmd.arg(cam_index.to_string());
	cmd.arg("-c");
	cmd.arg(format!("{property}={value}"));

	let mut child = cmd
		.spawn()
		.map_err(|e| CameraSetupError::ConfigError(e.to_string()))?;
	if let Err(e) = child.try_wait() {
		return Err(CameraSetupError::ConfigError(e.to_string()));
	}

	Ok(())
}

pub fn apply_v4l_conf(cam_index: u8, config: &CameraConfig) -> Result<(), CameraSetupError> {
	if let Some(exposure) = config.exposure {
		// A couple aliases for these
		v4l2_conf(cam_index, "exposure_auto", "1")?;
		v4l2_conf(cam_index, "auto_exposure", "1")?;
		v4l2_conf(cam_index, "exposure_absolute", &exposure.to_string())?;
		v4l2_conf(cam_index, "exposure_time_absolute", &exposure.to_string())?;
	}
	if !config.manual_brightness {
		if let Some(brightness) = config.brightness {
			v4l2_conf(cam_index, "brightness", &brightness.to_string())?;
		}
	}
	if !config.manual_contrast {
		if let Some(contrast) = config.contrast {
			v4l2_conf(cam_index, "contrast", &contrast.to_string())?;
		}
	}

	Ok(())
}
