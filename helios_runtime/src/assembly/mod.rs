//! Config-to-pipeline assembly: turns a resolved [`AutonomyStackConfig`] into a
//! ready-to-tick [`AutonomyPipeline`].
//!
//! - `stages` — [`build_pipeline`], the single entry point: builds the
//!   `[nodes]` entries, runs the seam passes and hands the result to the
//!   pipeline builder.
//! - `sensor_inputs` — where a node's sensor inputs may come from, and the
//!   check that each one has a source.
//! - `registry` — [`AutonomyRegistry`]: the node map and the extension store
//!   other tables live in (the command seam's [`CommandTypes`], the
//!   estimator's component tables); [`DuplicateKind`].
//! - `factory` — [`BuildContext`], [`FactoryOutput`], [`BuildFailure`] and
//!   [`FactoryError`]: the interface every node-kind factory shares.
//! - `component` — `ComponentTable`, the kind → factory table a node factory
//!   draws one of its parts from (an estimator's filter, dynamics and
//!   measurement models), and [`ComponentError`]. The tables themselves live
//!   in the extension of the concept that owns them.
//! - `instantiate` — builds every `[nodes]` entry through the factory
//!   registered for its kind.
//! - `config` — [`AutonomyStackConfig`], the stack's root config, one field per
//!   section, and [`AgentBaseConfig`], the portable agent profile.
//! - `seams` — the passes that combine several nodes' outputs into one seam
//!   channel, each from its stack section, with that section's config.
//! - `error` — [`PipelineAssemblyError`].
//! - `test_stub` — a do-nothing node for this module's tests.
//!
//! [`AutonomyPipeline`]: crate::pipeline::AutonomyPipeline

mod component;
mod config;
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
pub use self::config::{AgentBaseConfig, AutonomyStackConfig};
pub use self::error::PipelineAssemblyError;
pub use self::factory::{BuildContext, BuildFailure, FactoryError, FactoryOutput};
pub use self::registry::{AutonomyRegistry, DuplicateKind};
pub use self::seams::actuators::ActuatorSeamConfig;
pub use self::seams::command::{CommandFoldConfig, CommandTypes, DuplicateCommandType};
pub use self::seams::estimate::EstimateSeamConfig;
pub use self::seams::reference::{ArbitrationPolicyConfig, ReferenceSeamConfig};
pub use self::stages::build_pipeline;
