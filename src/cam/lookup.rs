use std::{
	path::{Path, PathBuf},
	process::Command,
};

/// Looks up a v4l camera ID (from /dev/v4l/by-id) to find it's index
pub fn lookup_camera_id_linux(camera_id: &str) -> std::io::Result<u32> {
	let path = PathBuf::from("/dev/v4l/by-id/").join(format!("{camera_id}-video-index0"));
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

/// Reads a symlink to find the target path
fn readlink(src: &Path) -> std::io::Result<PathBuf> {
	let mut command = Command::new("readlink");
	command.arg("-f").arg(src);
	let output = command.output()?;
	Ok(PathBuf::from(
		String::from_utf8_lossy(&output.stdout).trim(),
	))
}
