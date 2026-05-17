//! Autonomy pipeline orchestration — Bevy-free, portable to real hardware.
//!
//! Assembles `helios_core` algorithm stages into a [`pipeline::NodePipeline`] (to be
//! renamed `AutonomyPipeline`) that runs identically in simulation
//! and on hardware. Key types: `NodePipeline`, `NodePipelineBuilder`, `PipelineNode`,
//! `PortBus`, `AgentRuntime`.

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
pub use crate::pipeline::node_pipeline::MISSION_GOAL_INSTANCE;
pub use crate::pipeline::{NodePipeline, NodePipelineBuilder};
pub use crate::port::{ChannelKey, PortDescriptor};
pub use crate::stamped::{Health, Stamped};

pub use crate::config::{
    AckermannProcessNoiseConfig, AgentBaseConfig, AutonomyStack, ControllerConfig, EkfConfig,
    EkfDynamicsConfig, EstimatorConfig, SearchPlannerConfig, ImuProcessNoiseConfig,
    MapLayerConfig, MapperPoseSourceConfig, QuadcopterProcessNoiseConfig, UkfConfig,
};
pub use crate::runtime::AgentRuntime;
pub use crate::validation::{validate_autonomy_config, CapabilitySet, ConfigValidationError};
