// Re-export the entire Bevy prelude for convenience.
pub use bevy::prelude::*;

// Re-export the entire helios_core prelude so you can easily access
// pure types like `FrameHandle`, `Dynamics`, `Measurement`, etc.
pub use helios_core::prelude::*;

// The crate's public surface: types and plugins that sim modules, `helios_test`,
// and the examples import from here rather than reaching into deep module paths.
pub use crate::HeliosSimulationPlugin;

pub use crate::cli::Cli;

pub use crate::config::structs::{
    AgentConfig, AtmosphereConfig, RawScenarioConfig, ScenarioConfig, SensorConfig, TerrainConfig,
    Vehicle,
};
pub use crate::config::ConfigPlugin;

pub use crate::core::app_state::{AppState, SceneBuildSet, SimulationSet};
pub use crate::core::components::{
    BoundingBox3D, ControlOutputComponent, GroundTruthState, SemanticLabel, TerrainMedium,
    WorldObjectType,
};
pub use crate::core::host::{HeliosHost, Presentation};
pub use crate::core::spawn_requests::SpawnAgentConfigRequest;
pub use crate::core::transforms::{TfTree, TrackedFrame};

pub use crate::agents::sensors::imu::ImuPlugin;
pub use crate::agents::vehicles::ackermann::{
    AckermannAdapterComponent, AckermannCarPlugin, AckermannOutputAdapter,
};

pub use crate::brain_bridge::components::{AgentIdComponent, AutonomyPipelineComponent};
pub use crate::brain_bridge::{SensorPublishChannel, SensorPublisher};
