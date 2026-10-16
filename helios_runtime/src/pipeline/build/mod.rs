//! Building an [`AutonomyPipeline`](crate::AutonomyPipeline): everything that
//! runs once, inside [`PipelineBuilder::build`], and never during a tick.
//!
//! The builder works on declarations: nodes already built and [`NodePlan`]s,
//! both known by name and ports. `build` runs two checking stages on them,
//! then builds the planned nodes and assembles the pipeline:
//! - `wiring` checks names and suppliers from the declarations alone;
//! - `ordering` sorts the nodes into levels and, if it gets stuck, reports
//!   the loops `cycle` finds;
//! - `mounts` checks the mounts writers state and resolves each node
//!   output's mount in level order;
//! - `owners` checks every frame the body or a node owns has one owner;
//! - `builder` allocates bus slots from the declared ports, builds each plan
//!   with the mounts of its inputs, adds rate timers, and hands back the
//!   pipeline.
//!
//! Errors from every stage are [`PipelineBuildError`]s, defined in `error`.

mod builder;
mod cycle;
mod declaration;
mod error;
mod mounts;
mod ordering;
mod owners;
mod wiring;

pub use builder::PipelineBuilder;
pub(crate) use declaration::NodeDeclaration;
pub use declaration::{NodeConstructor, NodePlan};
pub use error::{ChannelUses, CycleEdge, FrameOwner, MountRefusal, PipelineBuildError};
pub use mounts::{InputMounts, MountLookupError};
