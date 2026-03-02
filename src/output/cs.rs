use std::{collections::HashMap, convert::Infallible, io::Cursor, time::Duration};

use bytes::Bytes;
use http_body_util::{combinators::BoxBody, BodyExt, Full, StreamBody};
use hyper::{
	body::Frame,
	header::{CACHE_CONTROL, CONTENT_TYPE, PRAGMA},
	server::conn::http1,
	service::service_fn,
	Response, StatusCode,
};
use hyper_util::rt::TokioIo;
use image::RgbImage;
use local_ip_address::local_ip;
use nt_client::{publish::NewPublisherError, ClientHandle};
use tokio::{
	net::TcpListener,
	sync::{broadcast, mpsc::Receiver},
};
use tracing::{error, info};

use crate::output::utils::{PubSub, ReconnectableClient};

/// Port for the MJPEG server
const CS_PORT: u16 = 1181;

/// Server for outputing camera data for NT clients to consume
pub struct CameraServer {
	frame_rx: Receiver<CameraServerInput>,
	/// Sender to streams for individual modules
	module_senders: HashMap<String, broadcast::Sender<RgbImage>>,
	/// List of publishers for streams
	streams_publishers: Vec<PubSub<Vec<String>>>,
	/// Reused allocation for the streams data
	streams: Vec<Vec<String>>,
}

impl CameraServer {
	pub async fn new(
		module_ids: &[String],
		client: &ClientHandle,
		reconnectable_client: &ReconnectableClient,
		frame_rx: Receiver<CameraServerInput>,
	) -> Result<Self, CameraServerError> {
		let mut streams = Vec::new();
		let mut streams_publishers = Vec::new();
		let mut module_senders = HashMap::new();
		let mut module_receivers = HashMap::new();

		let local_ip = local_ip()?;

		for module_id in module_ids {
			let table = format!("/CameraServer/Module {module_id}");

			let pubsub = reconnectable_client
				.get_topic(
					format!("{table}/streams"),
					Duration::from_millis(200),
					client,
				)
				.await?;
			streams_publishers.push(pubsub);

			let stream_url = format!("mjpeg:http://{local_ip}:{CS_PORT}/{module_id}");
			let module_streams = vec![stream_url];
			streams.push(module_streams);

			let (tx, rx) = broadcast::channel(5);
			module_senders.insert(module_id.clone(), tx);
			module_receivers.insert(module_id.clone(), rx);
		}

		let listener = TcpListener::bind(format!("0.0.0.0:{CS_PORT}"))
			.await
			.map_err(CameraServerError::TCPServer)?;
		info!("Started camera server HTTP");

		{
			tokio::spawn(async move {
				loop {
					let (stream, addr) = match listener.accept().await {
						Ok(result) => result,
						Err(e) => {
							error!("CameraServer connection failed: {e}");
							continue;
						}
					};
					info!("Accepted CameraServer connection from {addr}");

					let io = TokioIo::new(stream);
					let module_receivers: HashMap<_, _> = module_receivers
						.iter()
						.map(|(k, v)| (k.clone(), v.resubscribe()))
						.collect();
					if let Err(err) = http1::Builder::new()
						.serve_connection(
							io,
							service_fn(move |req| {
								let rx = if let Some(module) = req.uri().path().strip_prefix('/') {
									if let Some(rx) = module_receivers.get(module) {
										Some(rx.resubscribe())
									} else {
										None
									}
								} else {
									None
								};

								router(rx)
							}),
						)
						.await
					{
						error!("CS connection error: {err}");
					}
				}
			});
		}

		info!("Camera server connected");

		Ok(Self {
			streams_publishers,
			streams,
			module_senders,
			frame_rx,
		})
	}

	pub async fn run_forever(mut self) {
		loop {
			match self.frame_rx.recv().await {
				Some(input) => {
					self.run(input).await;
				}
				None => break,
			}
		}
	}

	async fn run(&mut self, input: CameraServerInput) {
		if let Some(sender) = self.module_senders.get(&input.module_id) {
			let _ = sender.send(input.frame);
		}

		// Update NT listing for streams
		for (pubsub, data) in self.streams_publishers.iter_mut().zip(self.streams.iter()) {
			pubsub.publish(data.clone()).await;
		}
	}
}

async fn router(
	module_rx: Option<broadcast::Receiver<RgbImage>>,
) -> Result<Response<BoxBody<Bytes, Infallible>>, Infallible> {
	if let Some(rx) = module_rx {
		Ok(mjpeg_stream(rx))
	} else {
		Ok(not_found())
	}
}

fn not_found() -> Response<BoxBody<Bytes, Infallible>> {
	Response::builder()
		.status(StatusCode::NOT_FOUND)
		.body(Full::new(Bytes::from("Not Found")).boxed())
		.unwrap()
}

fn mjpeg_stream(mut rx: broadcast::Receiver<RgbImage>) -> Response<BoxBody<Bytes, Infallible>> {
	let stream = async_stream::stream! {
		let mut frame_counter: u8 = 0;
		let mut jpeg_bytes = Vec::new();

		loop {
			let frame = match rx.recv().await {
				Ok(frame) => frame,
				Err(broadcast::error::RecvError::Closed) => break,
				Err(broadcast::error::RecvError::Lagged(..)) => continue,
			};
			frame_counter = frame_counter.wrapping_add(1);

			if convert_to_jpeg(frame, &mut jpeg_bytes).is_err() {
				continue;
			}

			let header = format!(
				"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
				jpeg_bytes.len(),
			);

			yield Ok::<Frame<Bytes>, Infallible>(Frame::data(Bytes::from(header)));
			yield Ok(Frame::data(Bytes::from(jpeg_bytes.clone())));
			yield Ok(Frame::data(Bytes::from("\r\n")));
		}
	};

	let body = StreamBody::new(stream);

	info!("Responded to Camera Server client");

	Response::builder()
		.header(
			CONTENT_TYPE,
			format!("multipart/x-mixed-replace; boundary=frame"),
		)
		.header(CACHE_CONTROL, "no-cache")
		.header(PRAGMA, "no-cache")
		.body(body.boxed())
		.unwrap()
}

/// Different errors for CameraServer setup
#[derive(thiserror::Error, Debug)]
pub enum CameraServerError {
	/// Failed to create publishers
	#[error("Failed to create publishers: {0}")]
	Publisher(#[from] NewPublisherError),
	/// Failed to get local IP address
	#[error("Failed to get IP address of this device: {0}")]
	LocalIp(#[from] local_ip_address::Error),
	/// Failed to start TCP server
	#[error("Failed to start TCP server: {0}")]
	TCPServer(std::io::Error),
	/// Encode failure
	#[error("Failed to encode frame: {0}")]
	Encode(#[from] image::ImageError),
}

/// Input to the CameraServer
pub struct CameraServerInput {
	pub frame: RgbImage,
	pub module_id: String,
}

fn convert_to_jpeg(image: RgbImage, buf: &mut Vec<u8>) -> Result<(), CameraServerError> {
	let mut cursor = Cursor::new(buf);
	image
		.write_to(&mut cursor, image::ImageFormat::Jpeg)
		.map_err(|e| CameraServerError::Encode(e))
}
