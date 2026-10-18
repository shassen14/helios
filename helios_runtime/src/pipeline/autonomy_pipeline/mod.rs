//! [`AutonomyPipeline`], the built graph, split by responsibility: the type
//! and its tick, the state kept per node, the recording of what hosts write,
//! and the methods a watching host calls.

mod host_writes;
mod pipeline;
mod scheduled;
mod tap;
mod watching;

pub use pipeline::{AutonomyPipeline, ReadMount};
pub use scheduled::{PIPELINE_LEAF_GROUP, TICK_DURATION_LEAF};

pub(in crate::pipeline) use host_writes::HostWrites;
pub(crate) use scheduled::observable_catalog;
pub(in crate::pipeline) use scheduled::ScheduledNode;
