use std::{
	collections::VecDeque,
	time::{Duration, Instant},
};

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

	/// Waits until this timer reaches an interval, then restarts it
	pub async fn wait_interval(&mut self, interval: Duration) {
		let check_interval = interval / 1000;
		loop {
			if self.interval(interval) {
				break;
			}
			tokio::time::sleep(check_interval).await;
		}
	}
}

/// Moving average of floats
pub struct MovingAverage {
	window: VecDeque<f64>,
	size: usize,
}

impl MovingAverage {
	pub fn new(size: usize) -> Self {
		Self {
			window: VecDeque::with_capacity(size),
			size,
		}
	}

	/// Adds a value and calculates the average
	pub fn calculate(&mut self, value: f64) -> f64 {
		self.window.push_back(value);
		if self.window.len() > self.size {
			self.window.pop_front();
		}

		self.window.iter().sum::<f64>() / self.size as f64
	}
}
