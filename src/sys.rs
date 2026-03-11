use std::{
	env::VarError,
	path::{Path, PathBuf},
	process::Command,
};

/// Generates the systemd service file
pub fn generate_service(config_path: &str) -> std::io::Result<String> {
	let executable_path = std::env::current_exe()?;
	let executable_path = executable_path.to_string_lossy();

	Ok(format!(
		r#"[Unit]
Description=AprilTag Vision System

[Service]
Type=simple
Restart=always
RestartSec=2
ExecStart={executable_path} --config {config_path}
Nice=1

[Install]
WantedBy=default.target"#
	))
}

/// Reads a symlink to find the target path
pub fn readlink(src: &Path) -> std::io::Result<PathBuf> {
	let mut command = Command::new("readlink");
	command.arg("-f").arg(src);
	let output = command.output()?;
	Ok(PathBuf::from(
		String::from_utf8_lossy(&output.stdout).trim(),
	))
}

/// Gets the path to the shared systemd service file
pub fn get_service_path() -> Result<PathBuf, VarError> {
	let home = std::env::var("HOME")?;
	Ok(PathBuf::from(home).join(".config/systemd/user/tag_vision.service"))
}
