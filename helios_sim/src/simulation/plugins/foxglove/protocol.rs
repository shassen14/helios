// helios_sim/src/simulation/plugins/foxglove/protocol.rs
//
// Foxglove WebSocket Protocol v1 wire types and frame encoding.

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Server → Client (JSON)
// ---------------------------------------------------------------------------

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ServerInfo {
    pub op: &'static str,
    pub name: String,
    pub capabilities: Vec<String>,
    pub supported_encodings: Vec<String>,
    pub session_id: String,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ChannelAdvertisement {
    pub id: u32,
    pub topic: String,
    pub encoding: String,
    pub schema_name: String,
    pub schema: String,
    pub schema_encoding: String,
}

#[derive(Debug, Serialize)]
pub struct AdvertiseMessage {
    pub op: &'static str,
    pub channels: Vec<ChannelAdvertisement>,
}

// ---------------------------------------------------------------------------
// Client → Server (JSON, parsed generically)
// ---------------------------------------------------------------------------

#[derive(Debug, Deserialize)]
pub struct SubscribeRequest {
    pub subscriptions: Vec<SubscriptionEntry>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SubscriptionEntry {
    pub id: u32,
    pub channel_id: u32,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnsubscribeRequest {
    pub subscription_ids: Vec<u32>,
}

// ---------------------------------------------------------------------------
// Internal channel types (no wire format)
// ---------------------------------------------------------------------------

/// Message sent from the Bevy system to the WebSocket server thread.
/// `Data` carries a serialized payload keyed by channel_id.
/// `Advertise` notifies connected clients of newly registered channels.
#[derive(Debug, Clone)]
pub enum BridgeMessage {
    Data {
        channel_id: u32,
        timestamp_ns: u64,
        payload: Vec<u8>,
    },
    Advertise(Vec<ChannelAdvertisement>),
}

/// Message sent from the WebSocket server back to the Bevy system.
#[derive(Debug)]
pub enum ServerControl {
    /// A client subscribed to `channel_id`.
    Subscribe { channel_id: u32 },
    /// A client unsubscribed from `channel_id`.
    Unsubscribe { channel_id: u32 },
}

// ---------------------------------------------------------------------------
// Binary frame encoding
// ---------------------------------------------------------------------------

/// Encodes a Foxglove WS "message data" binary frame.
///
/// Layout: `[0x01][4B subscription_id LE][8B timestamp_ns LE][payload]`
pub fn encode_message_frame(subscription_id: u32, timestamp_ns: u64, payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(1 + 4 + 8 + payload.len());
    frame.push(0x01);
    frame.extend_from_slice(&subscription_id.to_le_bytes());
    frame.extend_from_slice(&timestamp_ns.to_le_bytes());
    frame.extend_from_slice(payload);
    frame
}
