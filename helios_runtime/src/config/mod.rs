//! Portable autonomy configuration structs, shared by simulation and hardware.
//!
//! Re-exports `AgentBaseConfig`, `AutonomyStack`, and all sub-configs
//! (`EstimatorConfig`, `ControllerConfig`, `ReferenceSeamConfig`).
//! These structs are TOML-deserializable and contain zero Bevy or simulation types.

mod agent;
mod allocator;
mod autonomy;
mod command_space;
mod controller;
mod estimator;
mod reference;
mod tf;

pub use agent::AgentBaseConfig;
pub use allocator::AllocatorConfig;
pub(crate) use allocator::{STEER_POSITION_KIND, WHEEL_TORQUE_KIND};
pub use autonomy::AutonomyStack;
pub use command_space::CommandSpace;
pub use controller::ControllerConfig;
pub use controller::ControllerStateSourceConfig;
pub use controller::FoldRole;
pub(crate) use controller::{
    BICYCLE_STEER_KIND, DIRECT_TWIST_KIND, LONGITUDINAL_VELOCITY_KIND, ROAD_LOAD_KIND,
};
pub use estimator::{
    AckermannProcessNoiseConfig, AidingConfig, AugmentationConfig, EkfConfig, EkfDynamicsConfig,
    EkfInitialStateConfig, EstimatorConfig, IntegratedImuConfig, MockOracleEstimatorConfig,
    QuadcopterProcessNoiseConfig, SensorModelConfig, UkfConfig,
};
pub(crate) use estimator::{EKF_KIND, MOCK_ORACLE_KIND, UKF_KIND};
pub use reference::{ArbitrationPolicyConfig, ReferenceSeamConfig};
pub use tf::TfBufferConfig;
