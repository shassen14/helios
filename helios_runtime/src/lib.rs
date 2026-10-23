//! Autonomy pipeline orchestration — Bevy-free, portable to real hardware.
//!
//! Assembles `helios_core` algorithm stages into an [`AutonomyPipeline`] that runs
//! identically in simulation and on hardware. Key types: `AutonomyPipeline`,
//! `PipelineBuilder`, `PipelineNode`, `PortBus`.

pub mod assembly;
pub mod body;
pub mod channels;
pub mod diagnostics;
pub mod host;
pub mod nodes;
pub mod observe;
pub mod pipeline;
pub mod port;
pub mod prelude;
pub mod stamped;
pub mod tf_service;

pub use crate::body::{BodyCapabilities, BodyChannel};
pub use crate::host::HostDeclaration;
pub use crate::pipeline::node::{NodeId, PipelineNode, TickContext, HOST_PRODUCER_ID};
pub use crate::pipeline::{
    AutonomyPipeline, ChannelUses, CycleEdge, FrameConventions, FrameOwner, InputMounts,
    MountLookupError, MountRefusal, NodeConstructor, NodePlan, PinnedFrame, PipelineBuildError,
    PipelineBuilder, ReadMount,
};
pub use crate::port::{
    ChannelDescription, ChannelKey, ErasedStamped, InputNeed, InputPort, InputTiming, MountReading,
    MountRule, Pin, Pinned, PortDescriptor, Provenance, SlotVersion, Writer,
};
pub use crate::stamped::{Health, Stamped};

pub use crate::assembly::{
    build_pipeline, no_check, ActuatorSeamConfig, AgentBaseConfig, ArbitrationPolicyConfig,
    AutonomyRegistry, AutonomyStackConfig, BuildContext, BuildFailure, CommandFoldConfig,
    CommandTypes, ComponentError, DuplicateCommandType, DuplicateComponentKind, DuplicateKind,
    EstimateSeamConfig, FactoryError, FactoryOutput, NoParams, PipelineAssemblyError,
    ReferenceSeamConfig, Site,
};
pub use crate::nodes::estimation::{
    AssembledInput, DeclaredInput, DynamicsComponent, EstimatorInputBuilder, FilterParts,
    InputUnavailable, MeasurementFacts, SeedPose,
};
pub use crate::tf_service::TfBufferConfig;
