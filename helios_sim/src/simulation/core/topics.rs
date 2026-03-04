// helios_sim/src/simulation/core/topics.rs

use bevy::prelude::*;
use downcast_rs::{impl_downcast, Downcast};
use std::any::TypeId;
use std::collections::{HashMap, VecDeque};
use std::marker::PhantomData;
use std::sync::Arc;

const DEFAULT_TOPIC_CAPACITY: usize = 100;

// --- Trait for Type-Erased Topics ---
/// A trait to allow storing topics of different types in one HashMap.
pub trait AnyTopic: Downcast + Send + Sync {}
impl_downcast!(AnyTopic);

impl<T: Clone + Send + Sync + 'static> AnyTopic for Topic<T> {}

// --- Core Topic and Reader Structs ---

/// A message stored within a Topic, wrapping the data with a unique ID for cursor tracking.
#[derive(Clone, Debug)]
pub struct StampedMessage<T> {
    pub id: u64,
    pub message: T,
}

/// A generic, single-topic buffer. This is what's stored inside the TopicBus.
#[derive(Debug)]
pub struct Topic<T: Clone + Send + Sync + 'static> {
    buffer: VecDeque<StampedMessage<T>>,
    next_id: u64,
    capacity: usize,
}

impl<T: Clone + Send + Sync + 'static> Topic<T> {
    pub fn new(capacity: usize) -> Self {
        Self {
            buffer: VecDeque::with_capacity(capacity),
            next_id: 0,
            capacity,
        }
    }

    pub fn publish(&mut self, message: T) {
        if self.buffer.len() >= self.capacity {
            self.buffer.pop_front();
        }
        let stamped_message = StampedMessage {
            id: self.next_id,
            message,
        };
        self.buffer.push_back(stamped_message);
        self.next_id += 1;
    }
}

/// A component that acts as a cursor for a specific named topic.
#[derive(Component, Debug)]
pub struct TopicReader<T: 'static> {
    pub topic_name: String,
    last_id_read: Option<u64>,
    _phantom: PhantomData<T>,
}

impl<T: 'static> TopicReader<T> {
    pub fn new(topic_name: &str) -> Self {
        Self {
            topic_name: topic_name.to_string(),
            last_id_read: None,
            _phantom: PhantomData,
        }
    }

    pub fn read<'a, U: Clone + Send + Sync + 'static>(
        &mut self,
        topic: &'a Topic<U>,
    ) -> impl Iterator<Item = &'a StampedMessage<U>> {
        let start_index = match self.last_id_read {
            None => 0,
            Some(last_id) => topic
                .buffer
                .iter()
                .position(|msg| msg.id > last_id)
                .unwrap_or(topic.buffer.len()),
        };

        if let Some(newest_message) = topic.buffer.back() {
            self.last_id_read = Some(newest_message.id);
        }
        topic.buffer.range(start_index..)
    }
}

// --- Topic Metadata and Discovery ---

/// Metadata tag for classifying topics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TopicTag {
    /// Automatically assigned to topics created via auto-create on first publish.
    Generic,
    Imu,
    Gps,
    Lidar,
    Magnetometer,
    Odometry,
    GroundTruth,
}

/// A struct holding all information about a single topic on the bus.
pub struct TopicInfo {
    pub buffer: Box<dyn AnyTopic>,
    pub topic_type: TypeId,
    pub tag: TopicTag,
    pub owner_agent_name: String,
}

// --- The Main TopicBus Resource ---
#[derive(Resource, Default)]
pub struct TopicBus {
    topics: HashMap<String, TopicInfo>,
}

impl TopicBus {
    /// Explicitly create a topic with custom capacity, tag, and owner.
    /// Panics if the topic name already exists.
    pub fn create_topic<T: Clone + Send + Sync + 'static>(
        &mut self,
        name: &str,
        capacity: usize,
        tag: TopicTag,
        owner_agent_name: String,
    ) {
        if self.topics.contains_key(name) {
            panic!("Topic '{}' already exists!", name);
        }
        let topic_info = TopicInfo {
            buffer: Box::new(Topic::<T>::new(capacity)),
            topic_type: TypeId::of::<T>(),
            tag,
            owner_agent_name,
        };
        self.topics.insert(name.to_string(), topic_info);
    }

    /// Publish a value to a named topic.
    ///
    /// If the topic does not exist it is auto-created with `DEFAULT_TOPIC_CAPACITY`
    /// and `TopicTag::Generic`. If the topic exists but was created for a different
    /// type, the publish is dropped and a warning is emitted.
    pub fn publish<T: Clone + Send + Sync + 'static>(
        &mut self,
        topic_name: &str,
        message: T,
    ) -> bool {
        if !self.topics.contains_key(topic_name) {
            let topic_info = TopicInfo {
                buffer: Box::new(Topic::<T>::new(DEFAULT_TOPIC_CAPACITY)),
                topic_type: TypeId::of::<T>(),
                tag: TopicTag::Generic,
                owner_agent_name: String::new(),
            };
            self.topics.insert(topic_name.to_string(), topic_info);
        }

        if let Some(info) = self.topics.get_mut(topic_name) {
            if let Some(topic) = info.buffer.downcast_mut::<Topic<T>>() {
                topic.publish(message);
                return true;
            } else {
                warn!(
                    "TopicBus: type mismatch on topic '{}' — publish ignored.",
                    topic_name
                );
            }
        }
        false
    }

    /// Publish a pre-built `Arc<T>` to a named topic.
    ///
    /// Use this for large data (PointCloud, images) where multiple consumers should
    /// share the same allocation. Only the reference count is bumped per consumer read.
    pub fn publish_arc<T: Clone + Send + Sync + 'static>(
        &mut self,
        topic_name: &str,
        data: Arc<T>,
    ) -> bool {
        self.publish::<Arc<T>>(topic_name, data)
    }

    pub fn get_topic<T: Clone + Send + Sync + 'static>(
        &self,
        topic_name: &str,
    ) -> Option<&Topic<T>> {
        self.topics
            .get(topic_name)
            .and_then(|info| info.buffer.downcast_ref::<Topic<T>>())
    }

    pub fn find_topics_by_tag(&self, target_tag: TopicTag, target_agent_name: &str) -> Vec<String> {
        self.topics
            .iter()
            .filter(|(_name, info)| {
                info.tag == target_tag && info.owner_agent_name == target_agent_name
            })
            .map(|(name, _info)| name.clone())
            .collect()
    }
}

// --- Plugin ---

pub struct TopicBusPlugin;

impl Plugin for TopicBusPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TopicBus>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    fn read_all<T: Clone + Send + Sync + 'static>(
        reader: &mut TopicReader<T>,
        bus: &mut TopicBus,
    ) -> Vec<T> {
        // Re-borrow from bus inside a short scope so the mut borrow is released.
        bus.get_topic::<T>(&reader.topic_name.clone())
            .map(|topic| reader.read(topic).map(|m| m.message.clone()).collect())
            .unwrap_or_default()
    }

    // -------------------------------------------------------------------------
    // Auto-create on publish
    // -------------------------------------------------------------------------

    #[test]
    fn publish_auto_creates_topic() {
        let mut bus = TopicBus::default();
        let ok = bus.publish("test/topic", 42u32);
        assert!(ok, "publish should succeed");
        assert!(bus.get_topic::<u32>("test/topic").is_some());
    }

    #[test]
    fn publish_returns_false_on_type_mismatch() {
        let mut bus = TopicBus::default();
        bus.create_topic::<u32>("t", 10, TopicTag::Generic, String::new());
        // Publishing a different type to the same name must return false.
        let ok = bus.publish("t", "wrong type");
        assert!(!ok, "type mismatch should return false");
    }

    #[test]
    #[should_panic(expected = "already exists")]
    fn create_topic_panics_on_duplicate_name() {
        let mut bus = TopicBus::default();
        bus.create_topic::<u32>("dup", 10, TopicTag::Generic, String::new());
        bus.create_topic::<u32>("dup", 10, TopicTag::Generic, String::new());
    }

    // -------------------------------------------------------------------------
    // Ring buffer capacity
    // -------------------------------------------------------------------------

    #[test]
    fn ring_buffer_evicts_oldest_when_full() {
        let mut bus = TopicBus::default();
        bus.create_topic::<u32>("ring", 3, TopicTag::Generic, String::new());

        for i in 0u32..5 {
            bus.publish("ring", i);
        }

        // Capacity is 3, so only messages 2, 3, 4 remain.
        let mut reader = TopicReader::<u32>::new("ring");
        let msgs = read_all(&mut reader, &mut bus);
        assert_eq!(msgs, vec![2, 3, 4]);
    }

    #[test]
    fn auto_created_topic_uses_default_capacity() {
        let mut bus = TopicBus::default();
        // Publish DEFAULT_TOPIC_CAPACITY + 10 messages.
        for i in 0u32..DEFAULT_TOPIC_CAPACITY as u32 + 10 {
            bus.publish("cap", i);
        }
        let mut reader = TopicReader::<u32>::new("cap");
        let msgs = read_all(&mut reader, &mut bus);
        // Only the last DEFAULT_TOPIC_CAPACITY messages survive.
        assert_eq!(msgs.len(), DEFAULT_TOPIC_CAPACITY);
        assert_eq!(*msgs.first().unwrap(), 10u32);
        assert_eq!(*msgs.last().unwrap(), DEFAULT_TOPIC_CAPACITY as u32 + 9);
    }

    // -------------------------------------------------------------------------
    // TopicReader cursor tracking
    // -------------------------------------------------------------------------

    #[test]
    fn reader_cursor_tracks_new_messages_only() {
        let mut bus = TopicBus::default();
        bus.create_topic::<u32>("cursor", 10, TopicTag::Generic, String::new());
        let mut reader = TopicReader::<u32>::new("cursor");

        bus.publish("cursor", 1u32);
        bus.publish("cursor", 2u32);

        let first = read_all(&mut reader, &mut bus);
        assert_eq!(first, vec![1, 2], "should see first two messages");

        bus.publish("cursor", 3u32);

        let second = read_all(&mut reader, &mut bus);
        assert_eq!(second, vec![3], "should see only the new message");
    }

    #[test]
    fn reader_returns_empty_when_no_new_messages() {
        let mut bus = TopicBus::default();
        bus.create_topic::<u32>("idle", 10, TopicTag::Generic, String::new());
        let mut reader = TopicReader::<u32>::new("idle");

        bus.publish("idle", 1u32);
        let _ = read_all(&mut reader, &mut bus); // drain

        // No new messages published.
        let second = read_all(&mut reader, &mut bus);
        assert!(second.is_empty());
    }

    #[test]
    fn two_independent_readers_do_not_interfere() {
        let mut bus = TopicBus::default();
        bus.create_topic::<u32>("shared", 10, TopicTag::Generic, String::new());
        let mut reader_a = TopicReader::<u32>::new("shared");
        let mut reader_b = TopicReader::<u32>::new("shared");

        bus.publish("shared", 10u32);
        bus.publish("shared", 20u32);

        // Reader A drains.
        let a1 = read_all(&mut reader_a, &mut bus);
        assert_eq!(a1, vec![10, 20]);

        // Reader B hasn't read yet — should still see both.
        let b1 = read_all(&mut reader_b, &mut bus);
        assert_eq!(b1, vec![10, 20]);

        bus.publish("shared", 30u32);

        // Reader A sees only the new message.
        let a2 = read_all(&mut reader_a, &mut bus);
        assert_eq!(a2, vec![30]);

        // Reader B sees only the new message (its cursor was at 20).
        let b2 = read_all(&mut reader_b, &mut bus);
        assert_eq!(b2, vec![30]);
    }

    // -------------------------------------------------------------------------
    // Arc publishing (zero-copy path for heavy data)
    // -------------------------------------------------------------------------

    #[test]
    fn publish_arc_stores_shared_pointer() {
        let mut bus = TopicBus::default();
        let data: Arc<Vec<u8>> = Arc::new(vec![0u8; 1_000]);

        // Keep a weak reference to detect when the allocation is dropped.
        let weak = Arc::downgrade(&data);

        bus.publish_arc("heavy", data.clone()); // Arc clone: count = 2
        drop(data); // Drop our handle: count = 1 (still held by bus)

        // Allocation must still be alive (held by the ring buffer).
        assert!(weak.upgrade().is_some(), "bus should keep the data alive");

        // Consumers read an Arc clone — no Vec copy.
        let topic = bus.get_topic::<Arc<Vec<u8>>>("heavy").unwrap();
        let msg_arc = topic.buffer.back().unwrap().message.clone(); // cheap Arc clone
        assert_eq!(msg_arc.len(), 1_000);

        // Two strong references: one in bus ring, one in msg_arc.
        assert_eq!(Arc::strong_count(&msg_arc), 2);
    }

    #[test]
    fn publish_arc_type_is_arc_of_inner() {
        // `publish_arc<T>` stores `Arc<T>`, so `get_topic::<Arc<T>>` retrieves it.
        let mut bus = TopicBus::default();
        bus.publish_arc("pts", Arc::new(42u64));
        assert!(bus.get_topic::<Arc<u64>>("pts").is_some());
        // The raw type is NOT stored separately.
        assert!(bus.get_topic::<u64>("pts").is_none());
    }

    // -------------------------------------------------------------------------
    // Topic discovery
    // -------------------------------------------------------------------------

    #[test]
    fn find_topics_by_tag_filters_by_tag_and_owner() {
        let mut bus = TopicBus::default();
        bus.create_topic::<u32>("a", 10, TopicTag::Imu, "agent1".to_string());
        bus.create_topic::<u32>("b", 10, TopicTag::Gps, "agent1".to_string());
        bus.create_topic::<u32>("c", 10, TopicTag::Imu, "agent2".to_string());

        let mut imu_a1 = bus.find_topics_by_tag(TopicTag::Imu, "agent1");
        imu_a1.sort(); // HashMap iteration is unordered
        assert_eq!(imu_a1, vec!["a"]);

        let mut imu_a2 = bus.find_topics_by_tag(TopicTag::Imu, "agent2");
        imu_a2.sort();
        assert_eq!(imu_a2, vec!["c"]);

        assert!(bus.find_topics_by_tag(TopicTag::Lidar, "agent1").is_empty());
    }

    #[test]
    fn get_topic_returns_none_for_nonexistent() {
        let bus = TopicBus::default();
        assert!(bus.get_topic::<u32>("does/not/exist").is_none());
    }
}
