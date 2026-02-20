use std::{sync::Arc, time::Duration};

use nt_client::{
	data::NetworkTableData,
	publish::{NewPublisherError, Publisher},
	subscribe::{ReceivedMessage, Subscriber, SubscriptionOptions},
	topic::{Properties, Topic},
	Client, ClientHandle, NewClientOptions,
};
use tokio::sync::{
	broadcast::error::RecvError,
	mpsc::{Receiver, Sender},
	RwLock,
};

/// A client that can restart itself and recreate all of it's topics and pubsubs if it disconnects
pub struct ReconnectableClient {
	topics: RwLock<Vec<ReconnectableTopic>>,
}

impl ReconnectableClient {
	/// Spawns the tokio task for this automatically reconnecting client
	pub fn spawn(
		options: NewClientOptions,
		reconnect_interval: Duration,
		setup_fn: impl FnOnce(&Client, &Arc<ReconnectableClient>) + Send + 'static,
	) {
		tokio::spawn(async move {
			let mut client = Client::new(options.clone());
			let reconn_client = Arc::new(ReconnectableClient {
				topics: RwLock::new(Vec::new()),
			});
			let mut connect_result = client
				.connect_setup(|client| {
					println!("Client connected, setting up output");
					setup_fn(client, &reconn_client);
				})
				.await;

			// Reconnect logic
			loop {
				if let Err(e) = connect_result {
					eprintln!("Connection error: {e}. Reconnecting in {reconnect_interval:?}...");
					tokio::time::sleep(reconnect_interval).await;

					client = Client::new(options.clone());
					let reconn_client = reconn_client.clone();
					connect_result = client
						.connect_setup(move |client| {
							println!("Successfully reconnected!");
							let handle = client.handle().clone();
							tokio::spawn(
								async move { reconn_client.reconnect_topics(&handle).await },
							);
						})
						.await;
				} else {
					break;
				}
			}
		});
	}

	/// Gets a PubSub topic from the client
	pub async fn get_topic<T: NetworkTableData>(
		&self,
		topic_name: String,
		update_interval: Duration,
		client: &ClientHandle,
	) -> Result<PubSub<T>, NewPublisherError> {
		let topic = client.topic(&topic_name);

		let (tx, rx) = tokio::sync::mpsc::channel(3);
		let result = PubSub::with_reconnect_rx(topic, update_interval, Some(rx)).await?;

		self.topics.write().await.push(ReconnectableTopic {
			topic_name,
			reconnect_sender: tx,
		});

		Ok(result)
	}

	/// Reconnects topics
	async fn reconnect_topics(&self, client: &ClientHandle) {
		for topic in self.topics.read().await.iter() {
			let new_topic = client.topic(&topic.topic_name);
			let _ = topic.reconnect_sender.send(new_topic).await;
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
	publisher: Publisher<T>,
	subscriber: Subscriber,
	topic: Topic,
	connected: bool,
	update_interval: Duration,
	full_reconnect_rx: Option<Receiver<Topic>>,
}

impl<T: NetworkTableData> PubSub<T> {
	/// Creates a new PubSub from the given topic and the given full reconnect receiver
	async fn with_reconnect_rx(
		topic: Topic,
		update_interval: Duration,
		rx: Option<Receiver<Topic>>,
	) -> Result<Self, NewPublisherError> {
		let options = SubscriptionOptions {
			periodic: Some(update_interval),
			..Default::default()
		};

		let (publisher, subscriber) = Self::connect(&topic, options).await?;

		Ok(Self {
			publisher,
			subscriber,
			topic,
			connected: true,
			update_interval,
			full_reconnect_rx: rx,
		})
	}

	/// Publishes a value to this pubsub
	pub async fn publish(&mut self, value: T) {
		self.check_full_reconnect().await;

		// let _ = self.subscriber.recv().await;
		if self.publisher.set(value).await.is_err() {
			self.reconnect().await;
		}
	}

	/// Gets a value from this pubsub
	pub async fn get(&mut self) -> Option<T> {
		self.check_full_reconnect().await;

		match self.subscriber.recv().await {
			Ok(message) => {
				if let ReceivedMessage::Updated((_, value)) = message {
					T::from_value(value)
				} else {
					None
				}
			}
			Err(e) => {
				if let RecvError::Closed = e {
					self.reconnect().await;
				}
				None
			}
		}
	}

	/// Checks for a full client reconnect
	async fn check_full_reconnect(&mut self) {
		if let Some(rx) = &mut self.full_reconnect_rx {
			if let Ok(topic) = rx.try_recv() {
				self.topic = topic;
				self.reconnect().await;
			}
		}
	}

	/// Disconnects the pubsub and attempts to reconnect it. It may not succeed.
	pub async fn reconnect(&mut self) {
		if self.connected {
			eprintln!("Topic {} reconnecting...", self.topic.name());
		}

		self.connected = false;

		let options = SubscriptionOptions {
			periodic: Some(self.update_interval),
			..Default::default()
		};
		if let Ok((publisher, subscriber)) = Self::connect(&self.topic, options).await {
			self.publisher = publisher;
			self.subscriber = subscriber;
			self.connected = true;
		}
	}

	/// Connects to the publisher and subscriber
	async fn connect(
		topic: &Topic,
		sub_options: SubscriptionOptions,
	) -> Result<(Publisher<T>, Subscriber), NewPublisherError> {
		let Ok(subscriber) = topic.subscribe(sub_options).await else {
			return Err(NewPublisherError::Recv(
				tokio::sync::broadcast::error::RecvError::Closed,
			));
		};

		let publisher = topic.publish(Properties::default()).await?;

		Ok((publisher, subscriber))
	}
}
