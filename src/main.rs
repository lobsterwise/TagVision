use std::fs::File;

use clap::Parser;
use config::Config;
use runtime::Runtime;
use tracing::{error, info};

/// Camera hardware interfaces
mod cam;
/// Program configuration
mod config;
/// Computer vision, such as tag detection and pose estimation
mod cv;
/// Individual vision camera modules doing their own processing
mod module;
/// Output to NetworkTables and CameraServer
mod output;
/// Runtime for all of the modules, handling camera capture, processing, and output
mod runtime;
/// Overall system tests
#[cfg(test)]
mod test;
/// General utilities
mod util;

#[tokio::main(flavor = "multi_thread")]
async fn main() {
	tracing_subscriber::fmt::init();

	let config_file = File::open("config.json").expect("Failed to open config file");
	let mut config: Config =
		serde_json::from_reader(config_file).expect("Failed to parse config file");

	match Cli::try_parse() {
		Ok(cli) => {
			config.network.disabled |= cli.offline;
			if cli.local {
				config.network.address = Some("127.0.0.1".into());
			}

			for module in cli.disable {
				if let Some(module) = config.modules.get_mut(&module) {
					module.disabled = true;
				}
			}
		}
		Err(e) => {
			error!("CLI parse error: {e}");
		}
	}

	let runtime = Runtime::new(config).await;
	info!("Runtime initialized");

	runtime.run_forever().await;
}

#[derive(clap::Parser)]
struct Cli {
	/// Whether to disable the NetworkTables
	#[arg(long)]
	offline: bool,
	/// Whether to use 127.0.0.1 as the address for the NetworkTables
	#[arg(long)]
	local: bool,
	/// Modules to disable
	#[arg(long)]
	disable: Vec<String>,
}
