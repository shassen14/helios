//! State estimation traits and context types.
//!
//! The primary entry point for algorithm implementors is [`StateEstimator`].
//! Concrete filter implementations (`ExtendedKalmanFilter`, `UnscentedKalmanFilter`)
//! live in [`filters`].

use nalgebra::DVector;

use crate::frames::FrameAwareState;
use crate::prelude::{EstimationDynamics, MeasurementMessage};
use crate::types::TfProvider;
use std::any::Any;

/// World-context passed into every estimator predict/update call.
///
/// Created by the host layer (`helios_sim` or `helios_hw`) and borrowed for
/// the duration of one estimation tick. The `tf` field is `None` when no
/// transform tree is available (e.g. before the first physics tick); estimators
/// must handle this gracefully by skipping updates that require a TF lookup.
#[derive(Default)]
pub struct FilterContext<'a> {
    /// Provides access to the transform tree for coordinate frame conversions.
    /// Always check with `if let Some(tf) = context.tf` — never `.unwrap()`.
    pub tf: Option<&'a dyn TfProvider>,
}

pub struct EstimatorInputs {
    pub control: DVector<f64>,
}

/// Core contract for any state-estimation algorithm.
///
/// A `StateEstimator` maintains an internal [`FrameAwareState`] — a layout-typed
/// state vector with covariance — and exposes two operations:
///
/// 1. **`predict`** — propagates the state forward using dynamics and a control input.
/// 2. **`update`** — fuses an incoming sensor measurement to correct the estimate.
///
/// # Implementing a New Filter
///
/// 1. Create `estimation/filters/my_filter.rs` and implement this trait.
/// 2. Re-export from `estimation/filters/mod.rs`.
/// 3. Register a factory in `helios_sim`'s `AutonomyRegistry` — do not modify any spawning system.
///
/// Implementations must be `Send + Sync` because Bevy may run estimation systems
/// on any thread. Interior mutability (`Mutex`, `RefCell`) is not permitted — Bevy's
/// `&mut` access provides the necessary synchronization at the caller level.
pub trait StateEstimator: Send + Sync {
    /// Propagates the state and covariance forward by `dt` seconds.
    ///
    /// This call should linearize the dynamics, integrate the state vector, and
    /// propagate the covariance. On `dt <= 0`, the call must be a no-op.
    ///
    /// # Arguments
    /// * `dt` — elapsed time in seconds since the last prediction.
    /// * `u` — control input vector; dimension must match the dynamics model.
    /// * `context` — world context; `context.tf` may be `None`.
    fn predict(&mut self, dt: f64, inputs: &EstimatorInputs);

    /// Fuses a sensor measurement to correct the current estimate.
    ///
    /// Implementations must look up the appropriate measurement model via
    /// `message.sensor_handle`. If no matching model is registered or
    /// `context.tf` is `None`, silently skip and return.
    ///
    /// Never panics on mismatched sensor data — return early instead.
    fn update(&mut self, message: &MeasurementMessage, context: &FilterContext);

    /// Returns a reference to the current best state estimate.
    ///
    /// The returned [`FrameAwareState`] includes the state vector, covariance,
    /// and timestamp of the last update.
    fn get_state(&self) -> &FrameAwareState;

    /// Dynamic downcast to the concrete filter type, for algorithm-specific access.
    ///
    /// Use sparingly — prefer the trait interface. Required for registries that
    /// must inspect a `Box<dyn StateEstimator>` to retrieve tuning parameters.
    fn as_any_mut(&mut self) -> &mut dyn Any;

    /// Returns the dynamics model embedded in this filter.
    ///
    /// Used by `EstimationCore` to route sensor data to the correct control-input
    /// slot via `get_control_from_measurement`.
    fn get_dynamics_model(&self) -> &dyn EstimationDynamics;
}

pub mod filters;
