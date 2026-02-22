use tokio::sync::mpsc;
use tracing::info;

use crate::{
	cam::{Camera, CameraSetupError, FrameResult},
	config::{ModuleConfig, RuntimeConfig},
	cv::distort::OpenCVCameraIntrinsics,
};

/// An individual camera module
pub struct Module {}

impl Module {
	/// Initializes the module and camera
	pub fn init(
		id: String,
		config: ModuleConfig,
		runtime_config: &RuntimeConfig,
		output_tx: mpsc::Sender<ModuleOutput>,
		error_tx: mpsc::Sender<ModuleErrorOutput>,
	) -> Result<Self, CameraSetupError> {
		let intrinsics = OpenCVCameraIntrinsics::from_calib(&config.camera.intrinsics);

		{
			let runtime_config = runtime_config.clone();
			let intrinsics = intrinsics.clone();

			tokio::spawn(async move {
				let mut camera =
					Camera::from_config(config.camera.clone(), &runtime_config, config.max_errors);

				if camera.is_ok() {
					info!("Successfully initialized module {id}");
				}

				loop {
					match &mut camera {
						Ok(camera) => {
							let frame = camera.get_frame().await;

							let _ = output_tx.try_send(ModuleOutput {
								module_id: id.clone(),
								frame,
								intrinsics: intrinsics.clone(),
							});

							if let Some(err) = camera.self_check() {
								let _ = error_tx
									.send(ModuleErrorOutput {
										module_id: id.clone(),
										err,
									})
									.await;
								break;
							}
						}
						Err(err) => {
							let _ = error_tx
								.send(ModuleErrorOutput {
									module_id: id.clone(),
									err: err.clone(),
								})
								.await;
							break;
						}
					}
				}
			});
		}

		Ok(Self {})
	}
}

/// Output from a module, containing frame data and intrinsics
pub struct ModuleOutput {
	pub module_id: String,
	pub frame: FrameResult,
	pub intrinsics: OpenCVCameraIntrinsics,
}

/// Output from a module that needs a restart
pub struct ModuleErrorOutput {
	pub module_id: String,
	pub err: CameraSetupError,
}
