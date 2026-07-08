// helios_sim/src/prelude.rs

// Re-export the entire Bevy prelude for convenience.
pub use bevy::prelude::*;

// Re-export the entire helios_core prelude so you can easily access
// pure types like `FrameHandle`, `Dynamics`, `Measurement`, etc.
pub use helios_core::prelude::*;

// Re-export common simulationecific types for easy access in other plugins.
pub use crate::cli::Cli;
pub use crate::simulation::config::structs::{
    AgentConfig, RawScenarioConfig, ScenarioConfig, SensorConfig, Vehicle,
};
pub use crate::simulation::config::structs::{AtmosphereConfig, TerrainConfig};
pub use crate::simulation::core::app_state::{AppState, SceneBuildSet, SimulationSet};
pub use crate::simulation::core::components::{
    BoundingBox3D, SemanticLabel, TerrainMedium, WorldObjectType,
};
pub use crate::simulation::core::spawn_requests::SpawnAgentConfigRequest;
pub use crate::simulation::core::transforms::{TfTree, TrackedFrame};

pub use crate::simulation::plugins::autonomy::SensorPublishChannel;
pub use crate::simulation::plugins::sensors::imu::ImuPlugin;
pub use crate::simulation::plugins::vehicles::ackermann::{
    AckermannAdapterComponent, AckermannCarPlugin, AckermannOutputAdapter,
};

// host types
pub use crate::host::{HeliosHost, Presentation};
