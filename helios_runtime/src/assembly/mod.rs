//! Config-to-pipeline assembly: turns a resolved [`AutonomyStack`] into a
//! ready-to-tick [`AutonomyPipeline`].
//!
//! - `stages` — [`build_pipeline`], the single entry point, and the passes
//!   that build and wire each family's nodes.
//! - `sensor_inputs` — where a node's sensor inputs may come from, and the
//!   check that each one has a source.
//! - `registry` — [`AutonomyRegistry`], the kind-string → factory maps, and
//!   [`DuplicateKind`].
//! - `factory` — [`BuildContext`], [`FactoryOutput`] and [`FactoryError`]: the
//!   interface every node-kind factory shares.
//! - `contexts` — what each family's factory receives.
//! - `command` — node names and policy for the command and reference seams.
//! - `preprocessing` — builds preprocessing nodes from their config.
//! - `error` — [`PipelineAssemblyError`].
//!
//! [`AutonomyStack`]: crate::config::AutonomyStack
//! [`AutonomyPipeline`]: crate::pipeline::AutonomyPipeline

mod command;
pub mod contexts;
mod error;
mod factory;
mod preprocessing;
mod registry;
mod sensor_inputs;
mod stages;

pub use self::error::PipelineAssemblyError;
pub use self::factory::{BuildContext, FactoryError, FactoryOutput};
pub use self::registry::{AutonomyRegistry, DuplicateKind};
pub use self::stages::build_pipeline;
