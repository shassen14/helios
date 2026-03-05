// helios_sim/src/simulation/plugins/foxglove/mod.rs
//
// FoxgloveWebSocketPlugin — streams TopicBus data to Foxglove Studio via
// the Foxglove WebSocket Protocol v1.

pub mod bridge;
pub mod protocol;
pub mod serializers;
pub mod server;
pub mod types;

use bevy::prelude::*;
use std::sync::{Arc, Mutex, RwLock};
use tokio::sync::mpsc;

use crate::simulation::core::app_state::AppState;
use bridge::{ChannelRegistry, FoxgloveBridgeResource, FoxgloveTopicReaders, foxglove_bridge_system};

pub struct FoxgloveWebSocketPlugin {
    pub port: u16,
    pub enabled: bool,
}

impl Default for FoxgloveWebSocketPlugin {
    fn default() -> Self {
        Self {
            port: 8765,
            enabled: true,
        }
    }
}

impl Plugin for FoxgloveWebSocketPlugin {
    fn build(&self, app: &mut App) {
        if !self.enabled {
            return;
        }

        let port = self.port;

        // Create the Bevy→server channel (unbounded: sync send from Bevy, async recv in tokio).
        let (data_tx, data_rx) = mpsc::unbounded_channel::<protocol::BridgeMessage>();
        // Create the server→Bevy channel.
        let (ctrl_tx, ctrl_rx) = mpsc::unbounded_channel::<protocol::ServerControl>();

        // Shared channel snapshot: Bevy writes new channels; server reads on connect.
        let channel_snapshot: Arc<RwLock<Vec<protocol::ChannelAdvertisement>>> =
            Arc::new(RwLock::new(Vec::new()));

        // Spawn a dedicated OS thread that runs a single-threaded tokio runtime.
        // This is intentional: the WS server is a long-lived service, not a task pool job.
        let snapshot_for_server = channel_snapshot.clone();
        std::thread::Builder::new()
            .name("foxglove-ws-server".to_string())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("foxglove: failed to build tokio runtime");
                rt.block_on(server::run_server(port, data_rx, ctrl_tx, snapshot_for_server));
            })
            .expect("foxglove: failed to spawn server thread");

        app.insert_resource(FoxgloveBridgeResource {
            data_tx,
            ctrl_rx: Mutex::new(ctrl_rx),
            active_channels: Default::default(),
            channel_registry: ChannelRegistry::default(),
            channel_snapshot,
            last_warn_secs: 0.0,
        })
        .insert_resource(FoxgloveTopicReaders::default())
        .add_systems(
            Update,
            foxglove_bridge_system.run_if(in_state(AppState::Running)),
        );
    }
}
