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
	/// List of publishers for stream info
	info_pubs: HashMap<String, CameraInfoPubs>,
}

impl CameraServer {
	pub async fn new(
		module_ids: &[String],
		client: &ClientHandle,
		reconnectable_client: &ReconnectableClient,
		frame_rx: Receiver<CameraServerInput>,
	) -> Result<Self, CameraServerError> {
		let mut info_pubs = HashMap::new();
		let mut module_senders = HashMap::new();
		let mut module_receivers = HashMap::new();

		let local_ip = local_ip()?;

		for module_id in module_ids {
			let stream_url = format!("mjpg:http://{local_ip}:{CS_PORT}/{module_id}");

			let pubs = match CameraInfoPubs::new(
				module_id,
				stream_url,
				client,
				reconnectable_client,
			)
			.await
			{
				Ok(pubs) => pubs,
				Err(e) => {
					error!("Failed to initialize CS module info: {e}");
					continue;
				}
			};

			info_pubs.insert(module_id.clone(), pubs);

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

					tokio::spawn(async move {
						if let Err(err) = http1::Builder::new()
							.serve_connection(
								io,
								service_fn(move |req| {
									let rx =
										if let Some(module) = req.uri().path().strip_prefix('/') {
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
					});
				}
			});
		}

		info!("Camera server connected");

		Ok(Self {
			info_pubs,
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
		let dimensions = input.frame.dimensions();
		if let Some(sender) = self.module_senders.get(&input.module_id) {
			let _ = sender.send(input.frame);
		}

		// Update NT listing for streams
		for (module_id, pubs) in &mut self.info_pubs {
			if module_id == &input.module_id {
				pubs.set_mode(dimensions.0, dimensions.1);
			}
			pubs.publish().await;
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
			yield Ok(Frame::data(Bytes::copy_from_slice(&jpeg_bytes)));
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

/// Publishers for camera info
struct CameraInfoPubs {
	connected_pub: PubSub<bool>,
	streams_pub: PubSub<Vec<String>>,
	streams: Vec<String>,
	/// Stream format
	mode_pub: PubSub<String>,
	modes_pub: PubSub<Vec<String>>,
	mode: Option<String>,
}

impl CameraInfoPubs {
	async fn new(
		module_id: &str,
		stream_url: String,
		client: &ClientHandle,
		reconnectable_client: &ReconnectableClient,
	) -> Result<Self, NewPublisherError> {
		let table = format!("/CameraPublisher/Module {module_id}");
		let connected_pub = reconnectable_client
			.get_topic(format!("{table}/connected"), Duration::from_secs(1), client)
			.await?;
		let streams_pub = reconnectable_client
			.get_topic(format!("{table}/streams"), Duration::from_secs(1), client)
			.await?;
		let mode_pub = reconnectable_client
			.get_topic(format!("{table}/mode"), Duration::from_secs(1), client)
			.await?;
		let modes_pub = reconnectable_client
			.get_topic(format!("{table}/modes"), Duration::from_secs(1), client)
			.await?;

		Ok(Self {
			connected_pub,
			streams_pub,
			streams: vec![stream_url],
			mode_pub,
			modes_pub,
			mode: None,
		})
	}

	/// Publishes info
	async fn publish(&mut self) {
		let _ = self.connected_pub.publish(true).await;
		let _ = self.streams_pub.publish(self.streams.clone()).await;
		if let Some(mode) = &self.mode {
			let _ = self.mode_pub.publish(mode.clone()).await;
			let _ = self.modes_pub.publish(vec![mode.clone()]).await;
		}
	}

	/// Sets the camera mode when that info becomes available
	fn set_mode(&mut self, width: u32, height: u32) {
		if self.mode.is_none() {
			self.mode = Some(format!("{width}x{height} MJPEG 30 fps"));
		}
	}
}

fn convert_to_jpeg(image: RgbImage, buf: &mut Vec<u8>) -> Result<(), CameraServerError> {
	let mut cursor = Cursor::new(buf);
	image
		.write_to(&mut cursor, image::ImageFormat::Jpeg)
		.map_err(|e| CameraServerError::Encode(e))
}
