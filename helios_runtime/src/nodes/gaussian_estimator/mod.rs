//! Gaussian-estimator family: the node adapter, its bus-input assembly,
//! registration, and the dynamics + measurement-model building blocks the
//! factory composes. Wraps any `GaussianStateEstimator` (EKF, UKF, ESKF, IF);
//! non-Gaussian estimators (e.g. particle filters) are a separate family.
//!
//! - `node` — `GaussianEstimatorNode` plus the `AidingHandler` /
//!   `TypedAidingHandler` machinery its aiding updates run through.
//! - `input` — assembles filter inputs from the bus.
//! - `dynamics` / `measurement` — register the `EstimationDynamics` and
//!   `MeasurementModel` factories the estimator factory looks up by kind.
//! - `register` — registers the Gaussian filter factories (EKF, UKF).
//!
//! Unlike the other families, the assembler builds aiding handlers itself, so
//! `AidingHandler`, `TypedAidingHandler`, and the input builders are re-exported
//! for it. The `register` fn composes the family in dependency order: the leaf
//! dynamics and measurement factories before the estimators that consume them.

mod dynamics;
mod input;
mod measurement;
mod node;
mod register;

pub(crate) use input::{EstimatorInputBuilder, IntegratedImuInputBuilder};
pub(crate) use node::{AidingHandler, TypedAidingHandler};

use crate::registry::AutonomyRegistry;

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    dynamics::register(registry);
    measurement::register(registry);
    register::register(registry);
}
