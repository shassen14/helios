// helios_sim/src/prelude.rs

// Re-export the entire Bevy prelude for convenience.
pub use bevy::prelude::*;

// Re-export the entire helios_core prelude so you can easily access
// pure types like `FrameHandle`, `Dynamics`, `Measurement`, etc.
pub use helios_core::prelude::*;

// Re-export common simulation-specific types for easy access in other plugins.
pub use crate::simulation::config::structs::*;
pub use crate::simulation::core::app_state::{AppState, SceneBuildSet, SimulationSet};
pub use crate::simulation::core::components::{EstimationDynamicsModel, MeasurementModel};
pub use crate::simulation::core::spawn_requests::SpawnAgentConfigRequest;
pub use crate::simulation::core::transforms::{TfTree, TrackedFrame};

// Re-export the Bevy components that wrap the pure traits.
// pub use crate::simulation::plugins::estimation::ekf::{EkfCore, FilterState};
pub use crate::simulation::plugins::sensors::imu::ImuPlugin;
pub use crate::simulation::plugins::vehicles::ackermann::AckermannCarPlugin;
