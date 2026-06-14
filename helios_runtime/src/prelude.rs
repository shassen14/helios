//! High-frequency subset of the public API, for `use helios_runtime::prelude::*`.
//!
//! This is deliberately a *subset* of the full crate-root surface in `lib.rs`:
//! the handful of traits and nouns almost every node/host file needs in scope.
//! Config structs, registry, builders, and one-off types live in `lib.rs`
//! re-exports only — reach for them by full path. See
//! `docs/architecture/api_surface_conventions.md §4`.

pub use crate::pipeline::node::{PipelineNode, TickContext};
pub use crate::pipeline::{AutonomyPipeline, PipelineBuilder};
pub use crate::port::{ChannelKey, PortDescriptor};
pub use crate::runtime::AgentRuntime;
pub use crate::stamped::{Health, Stamped};
