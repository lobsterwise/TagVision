use std::{
	collections::{HashMap, HashSet},
	time::Duration,
};

use vision::{VisionRuntime, VisionThreadInput};

use crate::{
	config::{Config, DEFAULT_RECONNECT_INTERVAL, DEFAULT_UPDATE_RATE},
	cv::apriltag::layout::AprilTagLayout,
	module::Module,
	output::Output,
	util::Timer,
};

mod vision;

/// Runtime for the tag vision that manages camera modules, output, and threads
pub struct Runtime {
	config: Config,
	modules: HashMap<String, Module>,
	/// IDs modules that have yet to be initialized
	uninitialized_modules: HashSet<String>,
	/// Timer to periodically re-initialize dead modules
	module_init_timer: Timer,
	vision_runtime: VisionRuntime,
}

impl Runtime {
	/// Start the runtime. Modules will not be initialized until the runtime starts
	pub async fn new(config: Config) -> Self {
		let modules = config.modules.keys().cloned().collect();

		let layout = AprilTagLayout::load_from_preset(config.tags.layout);

		// Set up all the channels

		let (vision_runtime, output_receiver) =
			VisionRuntime::new(&config.detector_params, &config.tags, &layout);

		Output::new(
			output_receiver,
			config.network.clone(),
			config.runtime.clone(),
			&modules,
		)
		.await;

		Self {
			config,
			modules: HashMap::new(),
			uninitialized_modules: modules,
			vision_runtime,
			module_init_timer: Timer::new(),
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

				// Disabled modules
				if config.disabled {
					to_remove.insert(module_id.clone());
					continue;
				}

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

		// Get camera frames from modules and check for restarts
		for (module_id, module) in &mut self.modules {
			let intrinsics = module.get_intrinsics().clone();

			// Get and send frames
			let frames = module.get_frames();
			let frames = match frames {
				Ok(frames) => frames,
				Err(e) => {
					eprintln!("Failed to get frames from module '{module_id}': {e}");
					continue;
				}
			};

			for frame in frames {
				let frame = match frame {
					Ok(frame) => frame,
					Err(e) => {
						eprintln!("Frame error for module '{module_id}': {e}");
						continue;
					}
				};

				if let Err(e) = self
					.vision_runtime
					.send(VisionThreadInput {
						module: module_id.clone(),
						frame: frame.clone(),
						intrinsics: intrinsics.clone(),
					})
					.await
				{
					eprintln!("Vision thread not available: {e}");
				}
			}

			// Self check
			if module.self_check() {
				self.uninitialized_modules.insert(module_id.clone());
			}
		}
	}
}
