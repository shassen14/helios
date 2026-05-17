//! Node-based autonomy pipeline.
//!
//! [`AutonomyPipeline`] orchestrates a DAG of [`PipelineNode`]s exchanging values
//! through a typed [`PortBus`](crate::port::PortBus). One unified
//! [`tick`](AutonomyPipeline::tick) entry point runs every node whose
//! [`RateTimer`](rate_gate::RateTimer) is due in topological order.
//!
//! Construction goes through [`PipelineBuilder`]; the builder validates
//! the graph (cycles, unsatisfied inputs, duplicate producers) and returns
//! either a fully wired pipeline or a list of [`PipelineBuildError`]s.
//!
//! Algorithm-family node implementations live under [`nodes`]; reusable
//! per-node input/handler helpers live under [`builders`].

pub mod build_error;
pub mod builders;
pub mod node;
pub mod autonomy_pipeline;
pub mod nodes;
pub mod rate_gate;

pub use build_error::PipelineBuildError;
pub use node::{NodeId, PipelineNode, TickContext, HOST_PRODUCER_ID};
pub use autonomy_pipeline::{AutonomyPipeline, PipelineBuilder, MISSION_GOAL_INSTANCE};
