use std::{
	collections::{HashMap, HashSet},
	time::Duration,
};

use vision::{VisionRuntime, VisionThreadInput};

use crate::{
	config::{Config, DEFAULT_RECONNECT_INTERVAL, DEFAULT_UPDATE_RATE},
	module::Module,
	output::Output,
	util::Timer,
};

mod vision;

pub struct Runtime {
	config: Config,
	modules: HashMap<String, Module>,
	/// IDs modules that have yet to be initialized
	uninitialized_modules: HashSet<String>,
	/// Timer to periodically re-initialize modules
	module_init_timer: Timer,
	vision_runtime: VisionRuntime,
	output: Output,
}

impl Runtime {
	/// Start the runtime. Modules will not be initialized until the runtime starts
	pub fn new(config: Config) -> Self {
		let modules = config.modules.keys().cloned().collect();

		// Set up all the channels

		let (vision_runtime, output_receiver) =
			VisionRuntime::new(&config.detector_params, &config.runtime);

		let output = Output::new(
			output_receiver,
			config.network.clone(),
			config.runtime.clone(),
		);

		Self {
			config,
			modules: HashMap::new(),
			uninitialized_modules: modules,
			vision_runtime,
			module_init_timer: Timer::new(),
			output,
		}
	}

	/// Run the runtime forever
	pub async fn run_forever(mut self) {
		loop {
			self.run().await;
			tokio::time::sleep(Duration::from_secs_f32(
				self.config
					.runtime
					.update_rate
					.unwrap_or(DEFAULT_UPDATE_RATE)
					/ 1000.0,
			))
			.await;
		}
	}

	/// Run the runtime
	pub async fn run(&mut self) {
		// Attempt to start any modules that haven't yet
		let reconnect_interval = self
			.config
			.runtime
			.reconnect_interval
			.unwrap_or(DEFAULT_RECONNECT_INTERVAL);
		if self
			.module_init_timer
			.interval(Duration::from_secs_f32(reconnect_interval))
		{
			let mut to_remove = HashSet::new();

			for module_id in &self.uninitialized_modules {
				let Some(config) = self.config.modules.get(module_id) else {
					eprintln!("Uninitialized module '{module_id}' does not exist");
					continue;
				};

				let result = Module::init(config.clone(), &self.config.runtime);
				let module = match result {
					Ok(module) => module,
					Err(e) => {
						eprintln!("Failed to start module '{module_id}': {e}");
						continue;
					}
				};

				self.modules.insert(module_id.clone(), module);

				to_remove.insert(module_id.clone());
				println!("Successfully initialized module '{module_id}'");
			}

			// Remove the initialized modules from the uninitialized list
			for module in to_remove {
				self.uninitialized_modules.remove(&module);
			}
		}

		// Get camera frames from modules
		for (module_id, module) in &mut self.modules {
			let frames = module.get_frames();
			match frames {
				Ok(frames) => {
					for frame in frames {
						match frame {
							Ok(frame) => {
								if let Err(e) = self
									.vision_runtime
									.send(VisionThreadInput {
										module: module_id.clone(),
										frame,
									})
									.await
								{
									// eprintln!("Vision thread not available: {e}");
								}
							}
							Err(e) => {
								eprintln!("Frame error for module '{module_id}': {e}");
							}
						}
					}
				}
				Err(e) => {
					eprintln!("Failed to get frames from module '{module_id}': {e}");
				}
			}
		}
	}
}
