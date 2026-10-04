//! Building an [`AutonomyPipeline`](crate::AutonomyPipeline): everything that
//! runs once, inside [`PipelineBuilder::build`], and never during a tick.
//!
//! `build` runs two checking stages, then assembles the pipeline:
//! - `wiring` checks names and suppliers from the declarations alone;
//! - `ordering` sorts the nodes into levels and reports cycles;
//! - `builder` allocates bus slots and rate timers, logs the result through
//!   `dag_log`, and hands back the pipeline.
//!
//! Errors from either stage are [`PipelineBuildError`]s, defined in `error`.

mod builder;
mod dag_log;
mod error;
mod ordering;
mod wiring;

pub use builder::PipelineBuilder;
pub use error::{PipelineBuildError, Supplier};
