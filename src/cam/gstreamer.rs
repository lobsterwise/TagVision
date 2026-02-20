use std::{process::Command, sync::Arc};

use crossbeam_queue::ArrayQueue;
use gstreamer::{
	glib::{object::ObjectExt, ControlFlow},
	prelude::{ElementExt, GstObjectExt},
	Bus, MessageView,
};

use crate::config::{CameraConfig, RuntimeConfig, DEFAULT_QUEUE_SIZE};

use super::{CameraBackend, CameraFrame, CameraSetupError, CaptureError};

pub struct GStreamerCamera {
	bus: Bus,
	queue: Arc<ArrayQueue<Result<CameraFrame, CaptureError>>>,
}

impl CameraBackend for GStreamerCamera {
	fn init(
		config: &CameraConfig,
		runtime_config: &RuntimeConfig,
	) -> Result<Self, CameraSetupError> {
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
		if let Some(exposure) = config.exposure {
			v4l2_conf(&config.device_id, "exposure_auto", "1")?;
			v4l2_conf(
				&config.device_id,
				"exposure_absolute",
				&exposure.to_string(),
			)?;
		}
		if !config.manual_brightness {
			if let Some(brightness) = config.brightness {
				v4l2_conf(&config.device_id, "brightness", &brightness.to_string())?;
			}
		}
		if !config.manual_contrast {
			if let Some(contrast) = config.contrast {
				v4l2_conf(&config.device_id, "contrast", &contrast.to_string())?;
			}
		}

		// Start the pipeline
		if let Err(e) = pipeline.set_state(gstreamer::State::Playing) {
			return Err(CameraSetupError::StartError(e.to_string()));
		}

		// Set up the queue and the message handler for the bus

		let queue = Arc::new(ArrayQueue::new(
			runtime_config
				.camera_queue_size
				.unwrap_or(DEFAULT_QUEUE_SIZE) as usize,
		));

		// Error message queue
		{
			let queue = queue.clone();

			let result = bus.add_watch(move |_, msg| match msg.view() {
				MessageView::Error(err) => {
					let err = format!(
						"Error from {:?}: {} ({:?})",
						err.src().map(|s| s.path_string()),
						err.error(),
						err.debug()
					);

					queue.force_push(Err(CaptureError::GeneralError(err)));
					ControlFlow::Continue
				}
				_ => ControlFlow::Continue,
			});

			if let Err(e) = result {
				return Err(CameraSetupError::GeneralError(e.message.to_string()));
			}
		}

		// Appsink queue
		{
			let queue = queue.clone();
			pipeline.connect("appsink", true, |data| {
				// We don't process anything, this is the end of the line
				None
			});
		}

		Ok(Self { bus, queue })
	}

	fn get_frames(
		&mut self,
		buf: &mut Vec<Result<CameraFrame, CaptureError>>,
	) -> Result<(), CaptureError> {
		while let Some(frame) = self.queue.pop() {
			buf.push(frame);
		}

		Ok(())
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

/// Configures a camera property using v4l2-ctl
fn v4l2_conf(cam_id: &str, property: &str, value: &str) -> Result<(), CameraSetupError> {
	let mut cmd = Command::new("v4l2-ctl");
	cmd.arg("-d");
	cmd.arg(cam_id.to_string());
	cmd.arg("-c");
	cmd.arg(format!("{property}={value}"));

	let mut child = cmd
		.spawn()
		.map_err(|e| CameraSetupError::ConfigError(e.to_string()))?;
	if let Err(e) = child.try_wait() {
		return Err(CameraSetupError::ConfigError(e.to_string()));
	}

	Ok(())
}
