//! Node-based autonomy pipeline.
//!
//! [`NodePipeline`] orchestrates a DAG of [`PipelineNode`]s exchanging values
//! through a typed [`PortBus`](crate::port::PortBus). One unified
//! [`tick`](NodePipeline::tick) entry point runs every node whose
//! [`RateTimer`](rate_gate::RateTimer) is due in topological order.
//!
//! Construction goes through [`NodePipelineBuilder`]; the builder validates
//! the graph (cycles, unsatisfied inputs, duplicate producers) and returns
//! either a fully wired pipeline or a list of [`PipelineBuildError`]s.
//!
//! Algorithm-family node implementations live under [`nodes`]; reusable
//! per-node input/handler helpers live under [`builders`].
//!
//! Renamed to `AutonomyPipeline` / `PipelineBuilder` in #37 Step 9.

pub mod build_error;
pub mod builders;
pub mod node;
pub mod node_pipeline;
pub mod nodes;
pub mod rate_gate;

pub use build_error::PipelineBuildError;
pub use node::{NodeId, PipelineNode, TickContext, HOST_PRODUCER_ID};
pub use node_pipeline::{NodePipeline, NodePipelineBuilder, MISSION_GOAL_INSTANCE};
