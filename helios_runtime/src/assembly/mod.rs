//! Config-to-pipeline assembly: turns a resolved [`AutonomyStack`] into a
//! ready-to-tick [`AutonomyPipeline`].
//!
//! - `stages` — [`build_pipeline`], the single entry point, and the passes
//!   that build and wire each family's nodes.
//! - `sensor_inputs` — where a node's sensor inputs may come from, and the
//!   check that each one has a source.
//! - `registry` — [`AutonomyRegistry`], the kind-string → factory maps, the
//!   command-type table, [`DuplicateKind`] and [`DuplicateCommandType`].
//! - `factory` — [`BuildContext`], [`FactoryOutput`] and [`FactoryError`]: the
//!   interface every node-kind factory shares.
//! - `instantiate` — builds every `[nodes]` entry through the factory
//!   registered for its kind.
//! - `contexts` — what each family's factory receives.
//! - `seams` — the passes that combine several nodes' outputs into one seam
//!   channel, each from its stack section.
//! - `error` — [`PipelineAssemblyError`].
//! - `test_stub` — a do-nothing node for this module's tests.
//!
//! [`AutonomyStack`]: crate::config::AutonomyStack
//! [`AutonomyPipeline`]: crate::pipeline::AutonomyPipeline

pub mod contexts;
mod error;
mod factory;
mod instantiate;
mod registry;
mod seams;
mod sensor_inputs;
mod stages;
#[cfg(test)]
mod test_stub;

pub use self::error::PipelineAssemblyError;
pub use self::factory::{BuildContext, FactoryError, FactoryOutput};
pub use self::registry::{AutonomyRegistry, DuplicateCommandType, DuplicateKind};
pub use self::stages::build_pipeline;
