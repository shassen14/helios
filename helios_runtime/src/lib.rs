//! Autonomy pipeline orchestration — Bevy-free, portable to real hardware.
//!
//! Assembles `helios_core` algorithm stages into an [`pipeline::AutonomyPipeline`]
//! that runs identically in simulation and on hardware. Key types: `AutonomyPipeline`,
//! `PipelineBuilder`, `EstimationDriver`, `MapDriver`, `AgentRuntime`, `PipelineLevel`.

pub mod config;
pub mod estimation;
pub mod mapping;
pub mod pipeline;
pub mod port;
pub mod prelude;
pub mod runtime;
pub mod stage;
pub mod validation;
