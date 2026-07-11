// helios_sim/src/prelude.rs

// Re-export the entire Bevy prelude for convenience.
pub use bevy::prelude::*;

// Re-export the entire helios_core prelude so you can easily access
// pure types like `FrameHandle`, `Dynamics`, `Measurement`, etc.
pub use helios_core::prelude::*;

// Re-export common simulation-specific types for easy access in other plugins.
pub use crate::cli::Cli;
pub use crate::config::structs::{
    AgentConfig, RawScenarioConfig, ScenarioConfig, SensorConfig, Vehicle,
};
pub use crate::config::structs::{AtmosphereConfig, TerrainConfig};
pub use crate::core::app_state::{AppState, SceneBuildSet, SimulationSet};
pub use crate::core::components::{BoundingBox3D, SemanticLabel, TerrainMedium, WorldObjectType};
pub use crate::core::host::{HeliosHost, Presentation};
pub use crate::core::spawn_requests::SpawnAgentConfigRequest;
pub use crate::core::transforms::{TfTree, TrackedFrame};

pub use crate::brain_bridge::autonomy::SensorPublishChannel;
pub use crate::plugins::sensors::imu::ImuPlugin;
pub use crate::plugins::vehicles::ackermann::{
    AckermannAdapterComponent, AckermannCarPlugin, AckermannOutputAdapter,
};
