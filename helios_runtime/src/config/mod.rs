//! Portable autonomy configuration structs, shared by simulation and hardware.
//!
//! Re-exports `AgentBaseConfig`, `AutonomyStack`, and all sub-configs
//! (`EstimatorConfig`, `ControllerConfig`, `MapperConfig`, `PlannerConfig`, `SlamConfig`).
//! These structs are TOML-deserializable and contain zero Bevy or simulation types.

mod agent;
mod autonomy;
mod controller;
mod estimator;
mod mapper;
mod planner;
mod slam;

pub use agent::AgentBaseConfig;
pub use autonomy::{AutonomyStack, WorldModelConfig};
pub use controller::ControllerConfig;
pub use estimator::{
    AckermannProcessNoiseConfig, EkfConfig, EkfDynamicsConfig, EstimatorConfig,
    ImuProcessNoiseConfig, QuadcopterProcessNoiseConfig, UkfConfig,
};
pub use mapper::{MapperConfig, MapperPoseSourceConfig};
pub use planner::PlannerConfig;
pub use slam::{EkfSlamConfig, FactorGraphSlamConfig, SlamConfig};
