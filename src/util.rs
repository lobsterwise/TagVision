use std::time::{Duration, Instant};

/// A running timer
pub struct Timer {
	start_time: Instant,
}

impl Timer {
	pub fn new() -> Self {
		Self {
			start_time: Instant::now(),
		}
	}

	pub fn restart(&mut self) {
		self.start_time = Instant::now()
	}

	pub fn get_elapsed(&self) -> Duration {
		Instant::now() - self.start_time
	}

	pub fn has_elapsed(&self, duration: Duration) -> bool {
		self.get_elapsed() >= duration
	}

	pub fn interval(&mut self, interval: Duration) -> bool {
		if self.has_elapsed(interval) {
			self.restart();
			true
		} else {
			false
		}
	}
}
