//! Portable autonomy configuration structs, shared by simulation and hardware.
//!
//! Re-exports `AgentBaseConfig`, `AutonomyStack`, and its section configs
//! (`EstimateSeamConfig`, `ReferenceSeamConfig`, `CommandFoldConfig`,
//! `ActuatorSeamConfig`). A `[nodes]` kind keeps its own config beside its
//! factory.
//! These structs are TOML-deserializable and contain zero Bevy or simulation types.

mod actuators;
mod agent;
mod autonomy;
mod command;
mod estimate;
mod reference;
mod tf;

pub use actuators::ActuatorSeamConfig;
pub use agent::AgentBaseConfig;
pub use autonomy::AutonomyStack;
pub use command::CommandFoldConfig;
pub use estimate::EstimateSeamConfig;
pub use reference::{ArbitrationPolicyConfig, ReferenceSeamConfig};
pub use tf::TfBufferConfig;
