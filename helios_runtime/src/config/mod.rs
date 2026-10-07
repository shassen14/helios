//! Portable autonomy configuration structs, shared by simulation and hardware.
//!
//! Re-exports `AgentBaseConfig`, `AutonomyStack`, and all sub-configs
//! (`EstimatorConfig`, `ReferenceSeamConfig`, `CommandFoldConfig`,
//! `ActuatorSeamConfig`).
//! These structs are TOML-deserializable and contain zero Bevy or simulation types.

mod actuators;
mod agent;
mod autonomy;
mod command;
mod estimator;
mod reference;
mod tf;

pub use actuators::ActuatorSeamConfig;
pub use agent::AgentBaseConfig;
pub use autonomy::AutonomyStack;
pub use command::CommandFoldConfig;
pub use estimator::{
    AckermannProcessNoiseConfig, AidingConfig, AugmentationConfig, EkfConfig, EkfDynamicsConfig,
    EkfInitialStateConfig, EstimatorConfig, IntegratedImuConfig, MockOracleEstimatorConfig,
    QuadcopterProcessNoiseConfig, SensorModelConfig, UkfConfig,
};
pub(crate) use estimator::{EKF_KIND, MOCK_ORACLE_KIND, UKF_KIND};
pub use reference::{ArbitrationPolicyConfig, ReferenceSeamConfig};
pub use tf::TfBufferConfig;
