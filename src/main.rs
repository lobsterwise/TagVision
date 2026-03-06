use std::{fs::File, path::Path};

use clap::Parser;
use config::Config;
use runtime::Runtime;
use tracing::{error, info};

use crate::{
	cv::distort::OpenCVCameraIntrinsics,
	sys::{generate_service, get_service_path, readlink},
};

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
/// System interaction
mod sys;
/// Overall system tests
#[cfg(test)]
mod test;
/// General utilities
mod util;

#[tokio::main(flavor = "multi_thread")]
async fn main() {
	tracing_subscriber::fmt::init();

	let cli = match Cli::try_parse() {
		Ok(cli) => cli,
		Err(e) => {
			error!("CLI parse error: {e}");
			return;
		}
	};

	let config_path = cli.config.as_deref().unwrap_or("config.json");

	let config_file = File::open(config_path).expect("Failed to open config file");
	let mut config: Config =
		serde_json::from_reader(config_file).expect("Failed to parse config file");

	// Config modifications from CLI
	config.network.disabled |= cli.offline;
	if cli.local {
		config.network.address = Some("127.0.0.1".into());
	}

	for module in &cli.disable {
		if let Some(module) = config.modules.get_mut(module) {
			module.disabled = true;
		}
	}
	for module in &cli.enable {
		if let Some(module) = config.modules.get_mut(module) {
			module.disabled = false;
		}
	}

	// Commands
	if let Some(subcommand) = cli.subcommand {
		match subcommand {
			Command::Setup { config_path } => {
				// Convert the relative path to an absolute one
				let config_path =
					readlink(Path::new(&config_path)).expect("Failed to read link for config path");
				let service = generate_service(&config_path.to_string_lossy())
					.expect("Failed to generate service");

				let service_path = get_service_path().expect("Failed to get service path");
				std::fs::write(service_path, service).expect("Failed to write service file");

				return;
			}
			Command::Undistort {
				image,
				module,
				out_path,
			} => {
				let Some(module) = config.modules.get(&module) else {
					eprintln!("Module does not exist");
					return;
				};

				let intrinsics = OpenCVCameraIntrinsics::from_calib(&module.camera.intrinsics);

				let image = image::open(image).expect("Failed to open input image");
				let image = image.into();

				let new_image = intrinsics.unproject_image(image);
				new_image.save(out_path).expect("Failed to save image");
				println!("Image saved");

				return;
			}
		}
	}

	let runtime = Runtime::new(config).await;
	info!("Runtime initialized");

	runtime.run_forever().await;
}

#[derive(clap::Parser)]
struct Cli {
	/// Path to the config file, defaults to ./config.json
	#[arg(long)]
	config: Option<String>,
	/// Whether to disable the NetworkTables
	#[arg(long)]
	offline: bool,
	/// Whether to use 127.0.0.1 as the address for the NetworkTables
	#[arg(long)]
	local: bool,
	/// Modules to disable
	#[arg(long)]
	disable: Vec<String>,
	/// Modules to enable
	#[arg(long)]
	enable: Vec<String>,
	/// Subcommand
	#[command(subcommand)]
	subcommand: Option<Command>,
}

#[derive(clap::Subcommand)]
enum Command {
	/// Setup the systemd service
	Setup {
		/// Path to the config file to use. This should be a consistent path that you don't change.
		config_path: String,
	},
	/// Unproject an image file with the given camera's intrinsics
	Undistort {
		/// Path to the image file
		image: String,
		/// Module ID of the camera with the intrinsics
		module: String,
		/// Path to the output file
		out_path: String,
	},
}
