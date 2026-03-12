use std::{
	sync::{Arc, LazyLock},
	time::Duration,
};

use nt_client::{
	data::{DataType, NetworkTableData},
	publish::{NewPublisherError, Publisher},
	r#struct::{
		byte::{ByteBuffer, ByteReader},
		StructData,
	},
	subscribe::{ReceivedMessage, Subscriber, SubscriptionOptions},
	topic::{Properties, Topic},
	Client, ClientHandle, NewClientOptions,
};
use tokio::{
	select,
	sync::{
		mpsc::{self, Receiver, Sender},
		Mutex, RwLock,
	},
};
use tracing::{debug, error, info, warn};

/// Max timeout for NT publishes before cancelling
const SEND_TIMEOUT: Duration = Duration::from_secs(2);
/// Lock to prevent multiple pubsubs from connecting at the same time and mixing websocket messages
static CONNECT_LOCK: LazyLock<Arc<Mutex<()>>> = LazyLock::new(|| Arc::new(Mutex::new(())));

/// A client that can restart itself and recreate all of it's topics and pubsubs if it disconnects
pub struct ReconnectableClient {
	topics: RwLock<Vec<ReconnectableTopic>>,
}

impl ReconnectableClient {
	/// Spawns the tokio task for this automatically reconnecting client
	pub fn spawn(
		options: NewClientOptions,
		reconnect_interval: Duration,
		mut setup_fn: impl FnMut(&Client, &Arc<ReconnectableClient>) + Send + 'static + Clone,
	) {
		tokio::spawn(async move {
			let mut has_done_first_connection = false;
			let mut client = Client::new(options.clone());
			let reconn_client = Arc::new(ReconnectableClient {
				topics: RwLock::new(Vec::new()),
			});
			let mut connect_result = client
				.connect_setup(|client| {
					info!("NetworkTables connected, setting up output");
					setup_fn(client, &reconn_client);
					has_done_first_connection = true;
				})
				.await;

			// Reconnect logic
			loop {
				if let Err(e) = connect_result {
					error!("NetworkTables Connection error: {e}. Reconnecting in {reconnect_interval:?}...");
					tokio::time::sleep(reconnect_interval).await;

					client = Client::new(options.clone());
					let reconn_client = reconn_client.clone();
					let mut setup_fn = setup_fn.clone();
					connect_result = client
						.connect_setup(move |client| {
							info!("Successfully reconnected!");
							if !has_done_first_connection {
								info!("NetworkTables connected, setting up output");
								setup_fn(client, &reconn_client);
							} else {
								let handle = client.handle().clone();
								tokio::spawn(async move {
									reconn_client.reconnect_topics(&handle).await
								});
							}
						})
						.await;
				} else {
					break;
				}
			}
		});
	}

	/// Gets a PubSub topic from the client
	pub async fn get_topic<T: NetworkTableData + Send + Sync + 'static>(
		&self,
		topic_name: String,
		update_interval: Duration,
		client: &ClientHandle,
	) -> PubSub<T> {
		let topic = client.topic(&topic_name);

		let (tx, rx) = tokio::sync::mpsc::channel(3);
		let result = PubSub::with_reconnect_rx(topic, update_interval, rx);

		self.topics.write().await.push(ReconnectableTopic {
			topic_name,
			reconnect_sender: tx,
		});

		result
	}

	/// Reconnects topics
	async fn reconnect_topics(&self, client: &ClientHandle) {
		info!("Reconnecting topics");
		for topic in self.topics.read().await.iter() {
			let new_topic = client.topic(&topic.topic_name);
			if let Err(e) = topic.reconnect_sender.send(new_topic).await {
				error!("Failed to send topic reconnect: {e}");
			}
		}
	}
}

/// Topic with a sender to update the PubSub using it
struct ReconnectableTopic {
	topic_name: String,
	reconnect_sender: Sender<Topic>,
}

/// Wrapper for a NT4 publisher/subscriber that:
/// - Fixes the issue where you need a reading subscriber otherwise publishing won't work
/// - Automatically reconnects when disconnected
/// - Caches received values
pub struct PubSub<T: NetworkTableData> {
	publish_tx: Sender<T>,
	subscribe_rx: Receiver<T>,
}

impl<T: NetworkTableData + Send + Sync + 'static> PubSub<T> {
	/// Creates a new PubSub from the given topic and the given full reconnect receiver
	fn with_reconnect_rx(topic: Topic, update_interval: Duration, rx: Receiver<Topic>) -> Self {
		let options = SubscriptionOptions {
			periodic: Some(update_interval),
			..Default::default()
		};

		let (publish_tx, publish_rx) = mpsc::channel(20);
		let (subscribe_tx, subscribe_rx) = mpsc::channel(20);

		tokio::spawn(async move {
			let (publisher, subscriber) = match PubSubThread::connect(&topic, options.clone()).await
			{
				Ok((publisher, subscriber)) => (Some(publisher), Some(subscriber)),
				Err(e) => {
					error!(
						"Failed to do initial connection for pubsub {}: {e}",
						topic.name()
					);
					(None, None)
				}
			};

			let mut thread = PubSubThread {
				publish_rx,
				subscribe_tx,
				publisher,
				subscriber,
				topic,
				connected: true,
				update_interval,
				full_reconnect_rx: rx,
			};

			thread.run_forever().await;
		});

		Self {
			publish_tx,
			subscribe_rx,
		}
	}

	/// Publishes a value to this pubsub
	pub fn publish(&self, value: T) {
		let _ = self.publish_tx.try_send(value);
	}

	/// Gets a value from this pubsub, waiting for it.
	pub async fn get(&mut self) -> Option<T> {
		self.subscribe_rx.recv().await
	}

	/// Gets a value from this pubsub if it is available
	pub fn try_get(&mut self) -> Option<T> {
		self.subscribe_rx.try_recv().ok()
	}
}

struct PubSubThread<T: NetworkTableData + Send + Sync> {
	publish_rx: Receiver<T>,
	subscribe_tx: Sender<T>,
	full_reconnect_rx: Receiver<Topic>,
	publisher: Option<Publisher<T>>,
	subscriber: Option<Subscriber>,
	topic: Topic,
	connected: bool,
	update_interval: Duration,
}

impl<T: NetworkTableData + Send + Sync> PubSubThread<T> {
	async fn run_forever(&mut self) {
		loop {
			select! {
				value = self.publish_rx.recv() => {
					if let Some(value) = value {
						self.publish(value).await;
					} else {
						break;
					}
				}
				value = async { match &mut self.subscriber {
					Some(subscriber) => Some(subscriber.recv().await),
					None => None
				} }, if self.subscriber.is_some() => {
					let Some(value) = value else {
						return;
					};

					match value {
						Ok(message) => {
							if let ReceivedMessage::Updated((_, value)) = message {
								if let Some(value) = T::from_value(value) {
									let _ = self.subscribe_tx.try_send(value);
								}
							}
						}
						Err(e) => {
							error!("Subscription error: {e}");
						}
					}
				}
				new_topic = self.full_reconnect_rx.recv() => {
					if let Some(new_topic) = new_topic {
						self.topic = new_topic;
						self.reconnect().await;
					}
				}
			}
		}
	}

	/// Attempts to publish a value
	async fn publish(&mut self, value: T) {
		if let Some(publisher) = &mut self.publisher {
			select! {
				result = publisher.set(value) => {
					if let Err(e) = result {
						debug!("Publish failed: {e}");
					}
				}
				_ = tokio::time::sleep(SEND_TIMEOUT) => {
					warn!("Publish timed out");
				}
			}
		}
	}

	/// Disconnects the pubsub and attempts to reconnect it. It may not succeed.
	async fn reconnect(&mut self) {
		if self.connected {
			error!("Topic {} reconnecting...", self.topic.name());
		}

		self.connected = false;

		let options = SubscriptionOptions {
			periodic: Some(self.update_interval),
			..Default::default()
		};
		if let Ok((publisher, subscriber)) = Self::connect(&self.topic, options).await {
			self.publisher = Some(publisher);
			self.subscriber = Some(subscriber);
			self.connected = true;
		}
	}

	/// Connects to the publisher and subscriber
	async fn connect(
		topic: &Topic,
		sub_options: SubscriptionOptions,
	) -> Result<(Publisher<T>, Subscriber), NewPublisherError> {
		let _lock = CONNECT_LOCK.lock().await;

		let Ok(subscriber) = topic.subscribe(sub_options).await else {
			return Err(NewPublisherError::Recv(
				tokio::sync::broadcast::error::RecvError::Closed,
			));
		};

		let publisher = topic
			.publish(Properties {
				cached: Some(true),
				retained: Some(true),
				..Default::default()
			})
			.await?;

		Ok((publisher, subscriber))
	}
}

/// Struct array type publishable to NT
#[derive(Clone, Debug)]
pub struct StructArray<T>(pub Vec<T>);

impl<T: StructData + StructDataSize> NetworkTableData for StructArray<T> {
	fn data_type() -> DataType {
		DataType::StructArray(T::struct_type_name())
	}

	fn from_value(value: rmpv::Value) -> Option<Self> {
		match value {
			rmpv::Value::Binary(bytes) => {
				T::unpack_vec(&mut ByteReader::new(&bytes), bytes.len() / T::size()).map(Self)
			}
			_ => None,
		}
	}

	fn into_value(self) -> rmpv::Value {
		let mut buf = ByteBuffer::new();
		T::pack_iter(self.0, &mut buf);
		rmpv::Value::Binary(buf.into())
	}
}

/// Trait for NT StructData types that have a size
pub trait StructDataSize {
	fn size() -> usize;
}
