//! Autonomy pipeline orchestration — Bevy-free, portable to real hardware.
//!
//! Assembles `helios_core` algorithm stages into an [`AutonomyPipeline`] that runs
//! identically in simulation and on hardware. Key types: `AutonomyPipeline`,
//! `PipelineBuilder`, `PipelineNode`, `PortBus`.

pub mod assembly;
pub mod body;
pub mod channels;
pub mod config;
pub mod diagnostics;
pub mod nodes;
pub mod pipeline;
pub mod port;
pub mod prelude;
pub mod stamped;
pub mod tf_service;
pub mod validation;

pub use crate::body::{
    check_actuation_agreement, ActuatorKindMismatch, BodyCapabilities, Provenance, PublishedChannel,
};
pub use crate::pipeline::node::{NodeId, PipelineNode, TickContext, HOST_PRODUCER_ID};
pub use crate::pipeline::{
    AutonomyPipeline, CycleEdge, PipelineBuildError, PipelineBuilder, Supplier,
};
pub use crate::port::{
    ChannelKey, ErasedStamped, InputNeed, InputPort, InputTiming, PortDescriptor, SlotVersion,
};
pub use crate::stamped::{Health, Stamped};

pub use crate::assembly::contexts::{
    ControllerBuildContext, GaussianEstimatorBuildContext, MeasurementModelBuildContext,
};
pub use crate::assembly::{
    build_pipeline, AutonomyRegistry, BuildContext, DuplicateKind, FactoryError, FactoryOutput,
    PipelineAssemblyError,
};
pub use crate::config::{
    AckermannProcessNoiseConfig, AgentBaseConfig, AidingConfig, AutonomyStack, ControllerConfig,
    EkfConfig, EkfDynamicsConfig, EkfInitialStateConfig, EstimatorConfig, IntegratedImuConfig,
    QuadcopterProcessNoiseConfig, SensorModelConfig, UkfConfig,
};
pub use crate::validation::{validate_autonomy_config, CapabilitySet, ConfigValidationError};
