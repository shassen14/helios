// src/simulation/core/topics.rs

use bevy::prelude::*;
use downcast_rs::{Downcast, impl_downcast};
use nalgebra::{DMatrix, Isometry3, Vector3};
use std::any::TypeId;
use std::collections::{HashMap, VecDeque};
use std::marker::PhantomData;

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
    Imu,
    Gps,
    Lidar,
    // Add more tags as needed
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

    pub fn publish<T: Clone + Send + Sync + 'static>(
        &mut self,
        topic_name: &str,
        message: T,
    ) -> bool {
        if let Some(info) = self.topics.get_mut(topic_name) {
            if let Some(topic) = info.buffer.downcast_mut::<Topic<T>>() {
                topic.publish(message);
                return true;
            }
        }
        false
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

// --- Example Data Structures (replacing the old Event-based ones) ---

/// Topic: /sensor/imu/data
#[derive(Clone, Debug)]
pub struct ImuData {
    pub entity: Entity, // Keep the entity to know who this reading is for
    pub sensor_name: String,
    pub timestamp: f64,
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
}

// --- Events (Transient Messages) ---

/// Topic: /control/applied
/// The actual control input applied after actuator limits.
#[derive(Clone, Debug)]
pub struct AppliedControl {
    pub entity: Entity,
    pub timestamp: f64,
    pub control_vector: nalgebra::DVector<f64>,
}

// --- Components (Latched/Stateful Topics) ---

/// Topic: /state/estimated
/// The public, estimated pose of an agent.
#[derive(Component, Clone, Debug)]
pub struct EstimatedPose {
    pub timestamp: f64,
    pub pose: Isometry3<f64>,
    pub covariance: DMatrix<f64>,
}

/// Topic: /state/ground_truth
/// The perfect, ground truth state. Only for sensors and logging.
#[derive(Component, Clone, Debug)]
pub struct GroundTruthState {
    pub pose: Isometry3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub linear_acceleration: Vector3<f64>, // The value we will calculate
    pub last_linear_velocity: Vector3<f64>, // To help us calculate it
}

impl Default for GroundTruthState {
    fn default() -> Self {
        Self {
            pose: Isometry3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            linear_acceleration: Vector3::zeros(),
            last_linear_velocity: Vector3::zeros(),
        }
    }
}
