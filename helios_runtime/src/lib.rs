//! Autonomy pipeline orchestration — Bevy-free, portable to real hardware.
//!
//! Assembles `helios_core` algorithm stages into an [`AutonomyPipeline`] that runs
//! identically in simulation and on hardware. Key types: `AutonomyPipeline`,
//! `PipelineBuilder`, `PipelineNode`, `PortBus`, `AgentRuntime`.

pub mod assembler;
pub mod config;
pub mod pipeline;
pub mod port;
pub mod prelude;
pub mod registry;
pub mod runtime;
pub mod stamped;
pub mod validation;

pub use crate::pipeline::build_error::PipelineBuildError;
pub use crate::pipeline::node::{NodeId, PipelineNode, TickContext, HOST_PRODUCER_ID};
pub use crate::pipeline::autonomy_pipeline::MISSION_GOAL_INSTANCE;
pub use crate::pipeline::{AutonomyPipeline, PipelineBuilder};
pub use crate::port::{ChannelKey, PortDescriptor};
pub use crate::stamped::{Health, Stamped};

pub use crate::config::{
    AckermannProcessNoiseConfig, AgentBaseConfig, AidingConfig, AutonomyStack, ControllerConfig,
    EkfConfig, EkfDynamicsConfig, EkfInitialStateConfig, EstimatorConfig, IntegratedImuConfig,
    MapLayerConfig, MapperPoseSourceConfig, QuadcopterProcessNoiseConfig, SearchPlannerConfig,
    SensorModelConfig, UkfConfig,
};
pub use crate::assembler::{build_pipeline, PipelineAssemblyError};
pub use crate::runtime::AgentRuntime;
pub use crate::validation::{validate_autonomy_config, CapabilitySet, ConfigValidationError};
