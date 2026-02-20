use std::time::{Duration, Instant};

/// A running timer
pub struct Timer {
	start_time: Instant,
}

impl Timer {
	/// Creates a new timer that starts automatically
	pub fn new() -> Self {
		Self {
			start_time: Instant::now(),
		}
	}

	/// Restarts the timer to 0
	pub fn restart(&mut self) {
		self.start_time = Instant::now()
	}

	/// Gets the amount of time that has elapsed since the timer started
	pub fn get_elapsed(&self) -> Duration {
		Instant::now() - self.start_time
	}

	/// Checks if an amount of time has elapsed on the timer
	pub fn has_elapsed(&self, duration: Duration) -> bool {
		self.get_elapsed() >= duration
	}

	/// Checks if an amount of time has elapsed on the timer,
	/// and if it has, restarts the timer and returns true.
	/// Can be used to time repeating intervals.
	pub fn interval(&mut self, interval: Duration) -> bool {
		if self.has_elapsed(interval) {
			self.restart();
			true
		} else {
			false
		}
	}
}
