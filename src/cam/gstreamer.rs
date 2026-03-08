use std::process::Command;

use gstreamer::{
	glib::{object::ObjectExt, ControlFlow},
	prelude::{ElementExt, GstObjectExt},
	Bus, MessageView,
};
use tokio::sync::mpsc;

use crate::{
	cam::FrameResult,
	config::{CameraConfig, RuntimeConfig},
};

use super::{
	v4l::{apply_v4l_conf, lookup_camera_id_linux},
	CameraBackend, CameraSetupError, CaptureError,
};

pub struct GStreamerCamera {
	bus: Bus,
}

impl CameraBackend for GStreamerCamera {
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
		frame_tx: mpsc::Sender<FrameResult>,
	) -> Result<Self, CameraSetupError> {
		let _ = runtime_config;

		let result = gstreamer::init();
		if let Err(result) = result {
			return Err(CameraSetupError::LibraryInitError(result.to_string()));
		}

		// Create the pipeline
		let pipeline = create_pipeline(config);
		println!(
			"GStreamer pipeline for camera {}: {}",
			config.device_id, pipeline
		);
		let mut ctx = gstreamer::ParseContext::new();
		let pipeline = match gstreamer::parse::launch_full(
			&pipeline,
			Some(&mut ctx),
			gstreamer::ParseFlags::empty(),
		) {
			Ok(pipeline) => pipeline,
			Err(err) => {
				if let Some(gstreamer::ParseError::NoSuchElement) =
					err.kind::<gstreamer::ParseError>()
				{
					return Err(CameraSetupError::InvalidConfigError(format!(
						"Missing element(s): {:?}",
						ctx.missing_elements()
					)));
				} else {
					return Err(CameraSetupError::InvalidConfigError(format!(
						"Failed to parse pipeline: {err}"
					)));
				}
			}
		};
		let Some(bus) = pipeline.bus() else {
			return Err(CameraSetupError::GeneralError(
				"Pipeline did not return a bus".into(),
			));
		};

		// Configure the camera
		let cam_index = lookup_camera_id_linux(&config.device_id)
			.map_err(|e| CameraSetupError::General(e.to_string()))?;
		apply_v4l_conf(cam_index, config)?;

		// Start the pipeline
		if let Err(e) = pipeline.set_state(gstreamer::State::Playing) {
			return Err(CameraSetupError::StartError(e.to_string()));
		}

		let result = bus.add_watch(move |_, msg| match msg.view() {
			MessageView::Error(err) => {
				let err = format!(
					"Error from {:?}: {} ({:?})",
					err.src().map(|s| s.path_string()),
					err.error(),
					err.debug()
				);

				let _ = frame_tx.try_send(Err(CaptureError::GeneralError(err)));
				ControlFlow::Continue
			}
			_ => ControlFlow::Continue,
		});

		if let Err(e) = result {
			return Err(CameraSetupError::GeneralError(e.message.to_string()));
		}

		// Appsink queue
		{
			pipeline.connect("appsink", true, |data| {
				// We don't process anything, this is the end of the line
				None
			});
		}

		Ok(Self { bus })
	}
}

/// Creates the string passed to the gst_launch command
fn create_pipeline(config: &CameraConfig) -> String {
	// Use v4l2 to get the camera input
	let mut out = format!("v4l2src device=/dev/video{}", config.device_id);
	// Specify the video format
	out += &format!(
		" ! image/jpeg, width={}, height={}, format=MJPG, framerate={}/1",
		config.width, config.height, config.fps
	);
	// Decode the JPEG input
	out += " ! jpegdec ! videoconvert";
	// Convert to I420 and sink the output to the application
	out += " ! video/x-raw,format=I420 ! appsink max-buffers=1 drop=1";

	out
}
