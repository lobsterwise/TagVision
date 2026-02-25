use std::{
	collections::{HashMap, HashSet},
	time::Duration,
};

use tokio::sync::mpsc;
use tracing::{error, info};
use vision::{VisionRuntime, VisionThreadInput};

use crate::{
	config::Config,
	cv::apriltag::layout::AprilTagLayout,
	module::{Module, ModuleErrorOutput, ModuleOutput},
	output::Output,
	util::Timer,
};

mod vision;

/// Runtime for the tag vision that manages camera modules, output, and threads
pub struct Runtime {
	config: Config,
	modules: HashMap<String, Module>,
	module_rx: mpsc::Receiver<ModuleOutput>,
	module_tx: mpsc::Sender<ModuleOutput>,
	module_err_rx: mpsc::Receiver<ModuleErrorOutput>,
	module_err_tx: mpsc::Sender<ModuleErrorOutput>,
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

		let (module_tx, module_rx) = mpsc::channel(config.runtime.camera_queue_size as usize);
		let (module_err_tx, module_err_rx) = mpsc::channel(300);

		let layout = AprilTagLayout::load_from_preset(config.tags.layout);

		// Set up all the channels

		let (vision_runtime, output_receiver) = VisionRuntime::new(
			&config.runtime,
			&config.detector_params,
			&config.tags,
			&layout,
			&config.filters,
		);

		Output::new(
			output_receiver,
			config.network.clone(),
			&modules,
			layout.clone(),
		)
		.await;

		Self {
			config,
			modules: HashMap::new(),
			module_rx,
			module_tx,
			module_err_rx,
			module_err_tx,
			uninitialized_modules: modules,
			vision_runtime,
			module_init_timer: Timer::new(),
		}
	}

	/// Run the runtime forever
	pub async fn run_forever(mut self) {
		self.initialize_modules().await;

		loop {
			self.run().await;
		}
	}

	/// Attempt to initialize any non-running modules
	pub async fn initialize_modules(&mut self) {
		let mut to_remove = HashSet::new();

		for module_id in &self.uninitialized_modules {
			let Some(config) = self.config.modules.get(module_id) else {
				error!("Uninitialized module '{module_id}' does not exist");
				continue;
			};

			// Disabled modules
			if config.disabled {
				to_remove.insert(module_id.clone());
				continue;
			}

			let result = Module::init(
				module_id.clone(),
				config.clone(),
				&self.config.runtime,
				self.module_tx.clone(),
				self.module_err_tx.clone(),
			);
			let module = match result {
				Ok(module) => module,
				Err(e) => {
					error!("Failed to start module '{module_id}': {e}");
					continue;
				}
			};

			self.modules.insert(module_id.clone(), module);

			to_remove.insert(module_id.clone());
		}

		// Remove the initialized modules from the uninitialized list
		for module in to_remove {
			self.uninitialized_modules.remove(&module);
		}
	}

	/// Run the runtime
	pub async fn run(&mut self) {
		let reconnect_interval =
			Duration::from_secs_f32(self.config.runtime.camera_reconnect_interval);

		let module_restart_wait_task = async {
			loop {
				if self.module_init_timer.has_elapsed(reconnect_interval) {
					break;
				}
				tokio::time::sleep(Duration::from_secs_f32(0.25)).await;
			}
		};

		// Get and send frames, selecting to ensure that we don't block on getting frames
		// when the modules are all dead and we won't receive any
		tokio::select! {
			output = self.module_rx.recv() => {
				if let Some(output) = output {
					match output.frame {
						Ok(frame) => {
							if let Err(e) = self
								.vision_runtime
								.send(VisionThreadInput {
									module: output.module_id,
									frame: frame,
									intrinsics: output.intrinsics,
								})
								.await
							{
								error!("Vision thread not available: {e}");
							}
						}
						Err(e) => {
							error!("Frame error for module '{}': {e}", output.module_id);
						}
					};
				}
			}
			output = self.module_err_rx.recv() => {
				if let Some(output) = output {
					error!("Module error: {}", output.err);
					info!("Restarting module {}", output.module_id);
					self.uninitialized_modules.insert(output.module_id);
				}
			}
			() = module_restart_wait_task => {
				self.module_init_timer.restart();
				self.initialize_modules().await;
			}
		}
	}
}
