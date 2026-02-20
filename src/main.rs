use std::fs::File;

use config::Config;
use runtime::Runtime;

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
/// General utilities
mod util;

#[tokio::main(flavor = "multi_thread")]
async fn main() {
	let config_file = File::open("config.json").expect("Failed to open config file");
	let config: Config = serde_json::from_reader(config_file).expect("Failed to parse config file");

	let runtime = Runtime::new(config).await;
	println!("Runtime initialized");

	runtime.run_forever().await;
}
