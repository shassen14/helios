//! Config-to-pipeline assembly: turns a resolved [`AutonomyStack`] into a
//! ready-to-tick [`AutonomyPipeline`].
//!
//! - `stages` — [`build_pipeline`], the single entry point, and the passes
//!   that build and wire each family's nodes.
//! - `sensor_inputs` — where a node's sensor inputs may come from, and the
//!   check that each one has a source.
//! - `registry` — [`AutonomyRegistry`]: the node map, the legacy per-family
//!   maps, and the extension store other tables live in (the command seam's
//!   [`CommandTypes`], the estimator's component tables); [`DuplicateKind`].
//! - `factory` — [`BuildContext`], [`FactoryOutput`], [`BuildFailure`] and
//!   [`FactoryError`]: the interface every node-kind factory shares.
//! - `component` — `ComponentTable`, the kind → factory table a node factory
//!   draws one of its parts from (an estimator's filter, dynamics and
//!   measurement models), and [`ComponentError`]. The tables themselves live
//!   in the extension of the concept that owns them.
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

mod component;
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

pub(crate) use self::component::{BuiltComponent, ComponentTable};
pub use self::component::{ComponentError, DuplicateComponentKind, NoParams, Site};
pub use self::error::PipelineAssemblyError;
pub use self::factory::{BuildContext, BuildFailure, FactoryError, FactoryOutput};
pub use self::registry::{AutonomyRegistry, DuplicateKind};
pub use self::seams::command::{CommandTypes, DuplicateCommandType};
pub use self::stages::build_pipeline;
