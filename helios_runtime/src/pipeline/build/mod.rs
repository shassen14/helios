//! Building an [`AutonomyPipeline`](crate::AutonomyPipeline): everything that
//! runs once, inside [`PipelineBuilder::build`], and never during a tick.
//!
//! The builder works on declarations: nodes already built and [`NodePlan`]s,
//! both known by name and ports. `build` runs two checking stages on them,
//! then builds the planned nodes and assembles the pipeline:
//! - `wiring` checks names and suppliers from the declarations alone;
//! - `ordering` sorts the nodes into levels and, if it gets stuck, reports
//!   the loops `cycle` finds;
//! - `builder` allocates bus slots from the declared ports, builds each plan,
//!   adds rate timers, and hands back the pipeline.
//!
//! Errors from every stage are [`PipelineBuildError`]s, defined in `error`.

mod builder;
mod cycle;
mod declaration;
mod error;
mod ordering;
mod wiring;

pub use builder::PipelineBuilder;
pub(crate) use declaration::NodeDeclaration;
pub use declaration::{NodeConstructor, NodePlan};
pub use error::{ChannelUses, CycleEdge, PipelineBuildError};
