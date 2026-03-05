// helios_sim/src/simulation/plugins/foxglove/bridge.rs
//
// Bevy Resource and Update system that drives the Foxglove WebSocket bridge.

use std::any::TypeId;
use std::collections::{HashMap, HashSet};
use std::sync::Mutex;

use bevy::prelude::*;
// use helios_core::messages::PointCloud; // commented out — PointCloud not streamed to Foxglove
use helios_core::prelude::{FrameAwareState, MeasurementMessage};
use std::sync::Arc;

use crate::simulation::core::components::GroundTruthState;
use crate::simulation::core::topics::{TopicBus, TopicReader};
use crate::simulation::plugins::foxglove::protocol::{
    BridgeMessage, ChannelAdvertisement, ServerControl,
};
use crate::simulation::plugins::foxglove::serializers::{
    FRAME_AWARE_STATE_SCHEMA, GROUND_TRUTH_SCHEMA, MEASUREMENT_MESSAGE_SCHEMA,
    // POINT_CLOUD_SCHEMA,  // commented out — PointCloud not streamed to Foxglove
};
use crate::simulation::plugins::foxglove::types::{
    frame_aware_state_to_json, ground_truth_to_json, measurement_to_json,
    // point_cloud_to_json,  // commented out — PointCloud not streamed to Foxglove
};

// ---------------------------------------------------------------------------
// Channel registry
// ---------------------------------------------------------------------------

/// Tracks the mapping between topic names and Foxglove channel IDs.
#[derive(Default)]
pub struct ChannelRegistry {
    pub topic_to_channel: HashMap<String, u32>,
    pub channel_to_topic: HashMap<u32, String>,
    pub channel_type: HashMap<u32, TypeId>,
    next_id: u32,
}

impl ChannelRegistry {
    /// Register a topic and return `(channel_id, is_new)`.
    pub fn register(&mut self, topic_name: &str, type_id: TypeId) -> (u32, bool) {
        if let Some(&id) = self.topic_to_channel.get(topic_name) {
            return (id, false);
        }
        let id = self.next_id;
        self.next_id += 1;
        self.topic_to_channel.insert(topic_name.to_string(), id);
        self.channel_to_topic.insert(id, topic_name.to_string());
        self.channel_type.insert(id, type_id);
        (id, true)
    }
}

// ---------------------------------------------------------------------------
// Resources
// ---------------------------------------------------------------------------

/// All per-type TopicReaders maintained by the foxglove bridge.
#[derive(Resource, Default)]
pub struct FoxgloveTopicReaders {
    pub measurement: HashMap<String, TopicReader<MeasurementMessage>>,
    // pub point_cloud: HashMap<String, TopicReader<Arc<PointCloud>>>,  // no Foxglove 3D panel
    pub ground_truth: HashMap<String, TopicReader<GroundTruthState>>,
    pub frame_aware_state: HashMap<String, TopicReader<FrameAwareState>>,
}

/// The main bridge resource held by Bevy.
#[derive(Resource)]
pub struct FoxgloveBridgeResource {
    /// Send serialized frames to the WebSocket server thread.
    pub data_tx: tokio::sync::mpsc::UnboundedSender<BridgeMessage>,
    /// Receive subscription events from the WebSocket server thread.
    /// Wrapped in Mutex because UnboundedReceiver is Send but not Sync.
    pub ctrl_rx: Mutex<tokio::sync::mpsc::UnboundedReceiver<ServerControl>>,
    /// Channel IDs that at least one Foxglove client is subscribed to.
    pub active_channels: HashSet<u32>,
    /// Per-frame channel discovery state.
    pub channel_registry: ChannelRegistry,
    /// Shared snapshot of all registered channels, read by the server on each
    /// new client connection to send the initial `advertise` message.
    pub channel_snapshot: Arc<std::sync::RwLock<Vec<ChannelAdvertisement>>>,
    /// Monotonic time of last "queue full" warning (in elapsed secs).
    pub last_warn_secs: f64,
}

// ---------------------------------------------------------------------------
// Schema selection by TypeId
// ---------------------------------------------------------------------------

fn type_to_advertisement(
    channel_id: u32,
    topic_name: &str,
    type_id: TypeId,
) -> Option<ChannelAdvertisement> {
    let (schema_name, schema) = if type_id == TypeId::of::<MeasurementMessage>() {
        ("MeasurementMessage", MEASUREMENT_MESSAGE_SCHEMA)
    // } else if type_id == TypeId::of::<Arc<PointCloud>>() {
    //     ("PointCloud", POINT_CLOUD_SCHEMA)
    } else if type_id == TypeId::of::<GroundTruthState>() {
        ("GroundTruthState", GROUND_TRUTH_SCHEMA)
    } else if type_id == TypeId::of::<FrameAwareState>() {
        ("FrameAwareState", FRAME_AWARE_STATE_SCHEMA)
    } else {
        return None; // unsupported type — skip
    };

    Some(ChannelAdvertisement {
        id: channel_id,
        topic: topic_name.to_string(),
        encoding: "json".to_string(),
        schema_name: schema_name.to_string(),
        schema: schema.to_string(),
        schema_encoding: "jsonschema".to_string(),
    })
}

// ---------------------------------------------------------------------------
// Update system
// ---------------------------------------------------------------------------

pub fn foxglove_bridge_system(
    mut bridge: ResMut<FoxgloveBridgeResource>,
    topic_bus: Res<TopicBus>,
    mut readers: ResMut<FoxgloveTopicReaders>,
    time: Res<Time>,
) {
    // Explicitly deref ResMut so the borrow checker sees field-level borrows.
    let bridge: &mut FoxgloveBridgeResource = &mut *bridge;

    // 1. Drain ctrl_rx — collect messages first, then update fields.
    //    We collect into a local Vec so the MutexGuard is dropped before we
    //    touch active_channels.
    let ctrl_messages: Vec<ServerControl> = {
        let mut msgs = Vec::new();
        if let Ok(mut rx) = bridge.ctrl_rx.try_lock() {
            loop {
                match rx.try_recv() {
                    Ok(msg) => msgs.push(msg),
                    Err(_) => break,
                }
            }
        }
        msgs
    };
    for msg in ctrl_messages {
        match msg {
            ServerControl::Subscribe { channel_id } => {
                bridge.active_channels.insert(channel_id);
            }
            ServerControl::Unsubscribe { channel_id } => {
                bridge.active_channels.remove(&channel_id);
            }
        }
    }

    // 2. Lazy channel discovery — register any new topics appearing on the bus.
    let mut new_advertisements: Vec<ChannelAdvertisement> = Vec::new();
    for (topic_name, _tag, _owner, type_id) in topic_bus.list_topics() {
        let (channel_id, is_new) = bridge.channel_registry.register(&topic_name, type_id);
        if is_new {
            if let Some(adv) = type_to_advertisement(channel_id, &topic_name, type_id) {
                if type_id == TypeId::of::<MeasurementMessage>() {
                    readers
                        .measurement
                        .entry(topic_name.clone())
                        .or_insert_with(|| TopicReader::new(&topic_name));
                // } else if type_id == TypeId::of::<Arc<PointCloud>>() {
                //     readers.point_cloud.entry(topic_name.clone()).or_insert_with(|| TopicReader::new(&topic_name));
                } else if type_id == TypeId::of::<GroundTruthState>() {
                    readers
                        .ground_truth
                        .entry(topic_name.clone())
                        .or_insert_with(|| TopicReader::new(&topic_name));
                } else if type_id == TypeId::of::<FrameAwareState>() {
                    readers
                        .frame_aware_state
                        .entry(topic_name.clone())
                        .or_insert_with(|| TopicReader::new(&topic_name));
                }
                new_advertisements.push(adv);
            }
        }
    }

    if !new_advertisements.is_empty() {
        // Keep the shared snapshot current so new clients get all channels.
        if let Ok(mut snap) = bridge.channel_snapshot.write() {
            snap.extend(new_advertisements.clone());
        }
        let _ = bridge
            .data_tx
            .send(BridgeMessage::Advertise(new_advertisements));
    }

    // 3. Early return if no active subscriptions.
    if bridge.active_channels.is_empty() {
        return;
    }

    let timestamp_ns = (time.elapsed_secs_f64() * 1_000_000_000.0) as u64;
    let elapsed = time.elapsed_secs_f64();

    // 4. Publish data for each active channel.
    //    Snapshot so we can mutably borrow bridge fields inside the loop.
    let active_snapshot: Vec<u32> = bridge.active_channels.iter().copied().collect();

    for channel_id in active_snapshot {
        let topic_name = match bridge.channel_registry.channel_to_topic.get(&channel_id) {
            Some(n) => n.clone(),
            None => continue,
        };
        let type_id = match bridge.channel_registry.channel_type.get(&channel_id).copied() {
            Some(t) => t,
            None => continue,
        };

        // Serialize all new messages for this channel and enqueue frames.
        // Each branch borrows `bridge.data_tx` (immutable) and `bridge.last_warn_secs`
        // (mutable) via explicit field access — valid since `bridge` is `&mut`.
        if type_id == TypeId::of::<MeasurementMessage>() {
            if let Some(reader) = readers.measurement.get_mut(&topic_name) {
                if let Some(topic) = topic_bus.get_topic::<MeasurementMessage>(&topic_name) {
                    let payloads: Vec<_> = reader
                        .read(topic)
                        .filter_map(|s| serde_json::to_vec(&measurement_to_json(&s.message)).ok())
                        .collect();
                    for payload in payloads {
                        send_data(
                            &bridge.data_tx,
                            channel_id,
                            timestamp_ns,
                            payload,
                            elapsed,
                            &mut bridge.last_warn_secs,
                        );
                    }
                }
            }
        // Arc<PointCloud> branch commented out — no Foxglove 3D panel; belongs in MCAP logger.
        // } else if type_id == TypeId::of::<Arc<PointCloud>>() { ... }
        } else if type_id == TypeId::of::<GroundTruthState>() {
            if let Some(reader) = readers.ground_truth.get_mut(&topic_name) {
                if let Some(topic) = topic_bus.get_topic::<GroundTruthState>(&topic_name) {
                    let payloads: Vec<_> = reader
                        .read(topic)
                        .filter_map(|s| serde_json::to_vec(&ground_truth_to_json(&s.message)).ok())
                        .collect();
                    for payload in payloads {
                        send_data(
                            &bridge.data_tx,
                            channel_id,
                            timestamp_ns,
                            payload,
                            elapsed,
                            &mut bridge.last_warn_secs,
                        );
                    }
                }
            }
        } else if type_id == TypeId::of::<FrameAwareState>() {
            if let Some(reader) = readers.frame_aware_state.get_mut(&topic_name) {
                if let Some(topic) = topic_bus.get_topic::<FrameAwareState>(&topic_name) {
                    let payloads: Vec<_> = reader
                        .read(topic)
                        .filter_map(|s| {
                            serde_json::to_vec(&frame_aware_state_to_json(&s.message)).ok()
                        })
                        .collect();
                    for payload in payloads {
                        send_data(
                            &bridge.data_tx,
                            channel_id,
                            timestamp_ns,
                            payload,
                            elapsed,
                            &mut bridge.last_warn_secs,
                        );
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Helper: send a Data frame, throttle warnings on closed channel.
// ---------------------------------------------------------------------------

fn send_data(
    data_tx: &tokio::sync::mpsc::UnboundedSender<BridgeMessage>,
    channel_id: u32,
    timestamp_ns: u64,
    payload: Vec<u8>,
    elapsed_secs: f64,
    last_warn_secs: &mut f64,
) {
    if data_tx
        .send(BridgeMessage::Data {
            channel_id,
            timestamp_ns,
            payload,
        })
        .is_err()
    {
        // Channel closed means server thread exited; warn at most once per 5s.
        if elapsed_secs - *last_warn_secs > 5.0 {
            warn!("[foxglove] Bridge data channel closed — server thread may have exited");
            *last_warn_secs = elapsed_secs;
        }
    }
}
