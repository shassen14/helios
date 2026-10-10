//! [`AutonomyPipeline`], the built graph, split by responsibility: the type
//! and its tick, the state kept per node, and the methods a watching host
//! calls.

mod pipeline;
mod scheduled;
mod watching;

pub use pipeline::AutonomyPipeline;
pub use scheduled::{PIPELINE_LEAF_GROUP, TICK_DURATION_LEAF};

pub(crate) use scheduled::observable_catalog;
pub(in crate::pipeline) use scheduled::ScheduledNode;
