//! Estimator components: the parts an estimator node is assembled from, each
//! chosen by its own `kind` and built through a component table in
//! [`EstimatorComponents`], the registry extension this module owns.
//!
//! - `filter` — the recursive filter (EKF, UKF), built around a seeded state and
//!   a dynamics model ([`FilterParts`]).
//! - `dynamics` — the process model and the input builder feeding its predict
//!   ([`DynamicsComponent`]); the two must agree on the input schema.
//! - `measurement` — a measurement model, the typed reader for its payload and
//!   its noise R; the estimator applies what the reader takes.
//! - `gravity` — the believed-gravity default dynamics and measurement models
//!   share.
//!
//! None is specific to one estimator family: a batch smoother would reuse the
//! dynamics and measurement components and replace only the loop.

mod components;
mod dynamics;
mod filter;
#[cfg(test)]
pub(crate) mod flat_position;
mod gravity;
mod measurement;

pub use components::EstimatorComponents;
pub use dynamics::{DynamicsComponent, EstimatorInputBuilder, SeedPose};
pub use filter::FilterParts;

#[cfg(test)]
pub(crate) use measurement::PayloadReader;
pub(crate) use measurement::{Measurement, MeasurementSource, MeasurementWiring};

use crate::assembly::AutonomyRegistry;

/// Adds the built-in filter, dynamics and measurement kinds to `registry`'s
/// [`EstimatorComponents`].
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    let components = registry.extension_mut::<EstimatorComponents>();
    filter::register(components);
    dynamics::register(components);
    measurement::register(components);
}
