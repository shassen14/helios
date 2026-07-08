//! Portable autonomy configuration structs, shared by simulation and hardware.
//!
//! Re-exports `AgentBaseConfig`, `AutonomyStack`, and all sub-configs
//! (`EstimatorConfig`, `ControllerConfig`, `MapLayerConfig`, `SearchPlannerConfig`).
//! These structs are TOML-deserializable and contain zero Bevy or simulation types.

mod agent;
mod autonomy;
mod controller;
mod estimator;
mod mapper;
mod path_following;
mod planner;

pub use agent::AgentBaseConfig;
pub use autonomy::AutonomyStack;
pub use controller::ControllerConfig;
pub use controller::ControllerStateSourceConfig;
pub use estimator::{
    AckermannProcessNoiseConfig, AidingConfig, EkfConfig, EkfDynamicsConfig, EkfInitialStateConfig,
    EstimatorConfig, IntegratedImuConfig, QuadcopterProcessNoiseConfig, SensorModelConfig,
    UkfConfig,
};
pub use mapper::{MapLayerConfig, MapperPoseSourceConfig};
pub use path_following::PathFollowingConfig;
pub use planner::SearchPlannerConfig;
