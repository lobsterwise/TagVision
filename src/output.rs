use std::{net::Ipv4Addr, str::FromStr, time::Duration};

use image::RgbImage;
use nt_client::NewClientOptions;
use tokio::sync::mpsc::Receiver;

use crate::{
	config::{NetworkConfig, RuntimeConfig, DEFAULT_UPDATE_RATE},
	cv::geom::PoseUpdate,
	util::Timer,
};

/// Handles NetworkTables and CameraServer
pub struct Output {}

impl Output {
	pub fn new(
		input: Receiver<VisionOutput>,
		network_config: NetworkConfig,
		runtime_config: RuntimeConfig,
	) -> Self {
		// FIXME: Handle error of neither being present
		let addr = if let Some(address) = &network_config.address {
			nt_client::NTAddr::Custom(Ipv4Addr::from_str(address).unwrap())
		} else if let Some(team_number) = network_config.team_number {
			nt_client::NTAddr::TeamNumber(team_number)
		} else {
			panic!("No address provided")
		};
		let client = nt_client::Client::new(NewClientOptions {
			addr,
			name: network_config.name.clone(),
			..Default::default()
		});

		let thread_data = OutputThread {
			input,
			client,
			network_config,
			runtime_config,
			fps_timer: Timer::new(),
			fps_event_count: 0,
		};

		tokio::spawn(thread_data.run_forever());

		Self {}
	}
}

/// Data contained on the output task
struct OutputThread {
	input: Receiver<VisionOutput>,
	client: nt_client::Client,
	network_config: NetworkConfig,
	runtime_config: RuntimeConfig,
	fps_timer: Timer,
	fps_event_count: u16,
}

impl OutputThread {
	async fn run_forever(mut self) {
		loop {
			self.run().await;
			// tokio::time::sleep(Duration::from_secs_f32(
			// 	self.runtime_config
			// 		.update_rate
			// 		.unwrap_or(DEFAULT_UPDATE_RATE)
			// 		/ 1000.0,
			// ))
			// .await;
		}
	}

	async fn run(&mut self) {
		if let Some(output) = self.input.recv().await {
			self.fps_event_count += 1;
		}

		// Calculate FPS
		let fps_interval = 0.2;
		if self
			.fps_timer
			.interval(Duration::from_secs_f32(fps_interval))
		{
			let fps = self.fps_event_count as f32 / fps_interval;
			self.fps_event_count = 0;
			println!("FPS: {fps:.3}");
		}
	}
}

/// Output from the vision thread and input to the output thread
pub struct VisionOutput {
	pub module: String,
	pub update: PoseUpdate,
	pub frame: Option<RgbImage>,
}
