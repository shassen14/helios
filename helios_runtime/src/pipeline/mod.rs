//! Node-based autonomy pipeline.
//!
//! [`AutonomyPipeline`] orchestrates a DAG of [`PipelineNode`]s exchanging values
//! through a typed [`PortBus`](crate::port::PortBus). One unified
//! [`tick`](AutonomyPipeline::tick) entry point runs every node whose
//! [`RateTimer`](rate_gate::RateTimer) is due in topological order.
//!
//! Construction goes through [`PipelineBuilder`], in [`build`]. The builder
//! checks the graph (unique node names, exactly one supplier per input, no
//! cycles) and returns either a fully wired pipeline or a list of
//! [`PipelineBuildError`]s.
//!
//! This module is the family-agnostic engine — the DAG, its nodes' shared
//! contracts, and the builder. Concrete algorithm-family nodes live in
//! [`crate::nodes`].

pub mod autonomy_pipeline;
pub mod build;
pub(crate) mod key_format;
pub mod node;
pub mod rate_gate;

pub use autonomy_pipeline::AutonomyPipeline;
pub use build::{PipelineBuildError, PipelineBuilder, Supplier};
pub use node::{NodeId, PipelineNode, TickContext, HOST_PRODUCER_ID};
