//! Portable autonomy configuration structs, shared by simulation and hardware.
//!
//! Re-exports `AgentBaseConfig`, `AutonomyStack`, and all sub-configs
//! (`EstimatorConfig`, `ControllerConfig`, `MapLayerConfig`, `SearchPlannerConfig`).
//! These structs are TOML-deserializable and contain zero Bevy or simulation types.

mod agent;
mod allocator;
mod arbitration;
mod autonomy;
mod command_space;
mod controller;
mod estimator;
mod mapper;
mod path_following;
mod planner;
mod teleop;

pub use agent::AgentBaseConfig;
pub use allocator::AllocatorConfig;
pub use arbitration::{ArbitrationPolicyConfig, CommandArbitrationConfig, CommandSource};
pub use autonomy::AutonomyStack;
pub use command_space::CommandSpace;
pub use controller::ControllerConfig;
pub use controller::ControllerStateSourceConfig;
pub use controller::FoldRole;
pub use estimator::{
    AckermannProcessNoiseConfig, AidingConfig, AugmentationConfig, EkfConfig, EkfDynamicsConfig,
    EkfInitialStateConfig, EstimatorConfig, IntegratedImuConfig, MockOracleEstimatorConfig,
    QuadcopterProcessNoiseConfig, SensorModelConfig, UkfConfig,
};
pub use mapper::{MapLayerConfig, MapperPoseSourceConfig};
pub use path_following::PathFollowingConfig;
pub use planner::SearchPlannerConfig;
pub use teleop::TeleopMapperConfig;
