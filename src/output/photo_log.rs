use std::{
	path::{Path, PathBuf},
	time::{Duration, SystemTime, UNIX_EPOCH},
};

use image::RgbImage;
use tracing::error;

use crate::{config::PhotoLoggingConfig, util::Timer};

/// Periodically logs photos from cameras to disk
pub struct PhotoLogger {
	log_timer: Timer,
	config: PhotoLoggingConfig,
}

impl PhotoLogger {
	pub fn new(config: PhotoLoggingConfig) -> Self {
		Self {
			log_timer: Timer::new(),
			config,
		}
	}

	pub fn log(&mut self, module_id: &str, frame: &RgbImage) -> std::io::Result<()> {
		if self.config.interval == 0.0
			|| !self
				.log_timer
				.interval(Duration::from_secs_f32(self.config.interval))
		{
			return Ok(());
		}

		let frame = frame.clone();
		let log_dir = PathBuf::from(&self.config.path);
		let module_id = module_id.to_string();
		let max_count = self.config.max_count;

		tokio::task::spawn_blocking(move || {
			save_photo(log_dir, module_id, frame, max_count);
		});

		Ok(())
	}
}

fn save_photo(log_dir: PathBuf, module_id: String, frame: RgbImage, max_log_count: u16) {
	if !log_dir.exists() {
		let _ = std::fs::create_dir_all(&log_dir);
	}
	let _ = clear_old_logs(&log_dir, max_log_count);

	let timestamp = SystemTime::now()
		.duration_since(UNIX_EPOCH)
		.unwrap_or_default()
		.as_millis();

	let path = log_dir.join(format!("{module_id}_{timestamp}.jpg"));

	if let Err(e) = frame.save_with_format(path, image::ImageFormat::Jpeg) {
		error!("Failed to log photo: {e}");
	}
}

fn clear_old_logs(log_dir: &Path, max_count: u16) -> std::io::Result<()> {
	let mut files = Vec::new();
	for entry in log_dir.read_dir()? {
		let Ok(entry) = entry else {
			continue;
		};

		let Ok(meta) = entry.metadata() else {
			continue;
		};

		if !meta.is_file() {
			continue;
		}

		files.push((entry.path(), meta.created().unwrap_or(UNIX_EPOCH)));
	}

	files.sort_by_key(|x| std::cmp::Reverse(x.1));

	for (file, _) in files.into_iter().skip(max_count as usize) {
		let _ = std::fs::remove_file(file);
	}

	Ok(())
}
