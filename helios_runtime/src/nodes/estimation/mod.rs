//! Estimator components: the parts an estimator node is assembled from, each
//! chosen by its own `kind` and built through a component table in
//! [`EstimatorComponents`], the registry extension this module owns.
//!
//! - `filter` — the recursive filter (EKF, …), built around a seeded state and
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

#![expect(dead_code, reason = "no node factory builds estimator components yet")]

mod components;
mod dynamics;
mod filter;
mod gravity;
mod measurement;

pub use components::EstimatorComponents;
pub use dynamics::{DynamicsComponent, EstimatorInputBuilder, SeedPose};
pub use filter::FilterParts;

pub(crate) use dynamics::{
    IntegratedImuInputBuilder, DEFAULT_ACCEL_BIAS_UNCERTAINTY_MPS2,
    DEFAULT_GYRO_BIAS_UNCERTAINTY_RADPS, DEFAULT_ORIENTATION_UNCERTAINTY_DEG,
    DEFAULT_POSITION_UNCERTAINTY_M, DEFAULT_VELOCITY_UNCERTAINTY_MPS, INTEGRATED_IMU_KIND,
};
pub(crate) use filter::EKF_FILTER_KIND;
pub(crate) use gravity::default_gravity_enu;
pub(crate) use measurement::{
    Measurement, PayloadReader, ACCELEROMETER_KIND, GPS_POSITION_KIND, GYROSCOPE_KIND,
    MAGNETOMETER_KIND,
};

use crate::assembly::AutonomyRegistry;

/// Adds the built-in filter, dynamics and measurement kinds to `registry`'s
/// [`EstimatorComponents`].
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    let components = registry.extension_mut::<EstimatorComponents>();
    filter::register(components);
    dynamics::register(components);
    measurement::register(components);
}
