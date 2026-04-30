//! Measurement model trait and concrete sensor implementations.
//!
//! Each sensor type implements [`Measurement`], which describes the function
//! `z = h(x) + v` — the ideal measurement predicted from filter state, plus noise.
//!
//! # Implementing a New Sensor Model
//!
//! 1. Create `models/estimation/measurement/my_sensor.rs`, implement [`Measurement`].
//! 2. `predict_measurement` must return `None` if `message.data` is the wrong variant — never panic.
//! 3. Re-export from this `mod.rs`.
//! 4. Register the sensor entity's handle and model in the EKF/UKF `measurement_models` map
//!    inside the corresponding `helios_sim` spawning system.

use crate::frames::{FrameAwareState, StateVariable};
use crate::prelude::{MeasurementData, MeasurementMessage};
use crate::types::TfProvider;
use dyn_clone::DynClone;
use nalgebra::{DMatrix, DVector};
use std::any::Any;
use std::fmt::Debug;

/// Mathematical model of a sensor: `z = h(x) + v`.
///
/// A `Measurement` implementation describes one physical sensor type — GPS,
/// IMU, magnetometer, etc. It is registered with a filter alongside the
/// sensor's [`FrameHandle`](crate::types::FrameHandle) so the filter can
/// dispatch measurements to the correct model in O(1).
///
/// `Measurement` objects are cloneable (`DynClone`) so the filter can be
/// deep-copied for multi-hypothesis tracking without re-building the model map.
pub trait Measurement: DynClone + Debug + Send + Sync {
    /// Layout of the measurement vector `z` expressed in the sensor's native frame.
    ///
    /// Used by higher-level tooling to introspect which state variables this
    /// sensor observes. Not used directly by EKF/UKF internals.
    fn get_measurement_layout(&self) -> Vec<StateVariable>;

    /// Converts a generic [`MeasurementData`] enum into the typed measurement vector `z`.
    ///
    /// This method is the primary dispatch gate: if the incoming data matches
    /// this model's sensor type, return `Some(z)`. Otherwise return `None` to
    /// indicate that this model does not apply.
    ///
    /// # Returns
    /// * `Some(z)` — this model handles the data; `z` has dimension `m`.
    /// * `None` — wrong sensor type; the filter will skip this model.
    fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>>;

    /// The measurement noise covariance matrix `R` (m × m).
    ///
    /// Built from `noise_stddev` entries in the sensor TOML config at spawn time.
    /// Never hardcode noise values here — they must come from config.
    fn get_r(&self) -> &DMatrix<f64>;

    /// Computes the ideal predicted measurement `z_pred = h(x)` from the filter state.
    ///
    /// This is called during the EKF/UKF update step to form the innovation
    /// `y = z - z_pred`. The `tf` argument provides sensor-to-world transforms
    /// needed for models like IMU that must rotate measurements into body frame.
    ///
    /// Return `None` if `message.data` is the wrong variant — never panic on mismatch.
    ///
    /// # Arguments
    /// * `filter_state` — current filter state `x`.
    /// * `message` — the incoming measurement event; use `message.data` to check variant.
    /// * `tf` — transform provider for frame conversions.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        tf: &dyn TfProvider,
    ) -> Option<DVector<f64>>;

    /// Computes the measurement Jacobian `H = ∂h/∂x` (m × n).
    ///
    /// Used by the EKF for linearization. For nonlinear measurement functions,
    /// use finite differences with adaptive epsilon `ε = 1e-5 * (1 + |xᵢ|)`.
    /// The UKF does not call this method.
    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        context: &dyn TfProvider,
    ) -> DMatrix<f64>;

    /// Dynamic downcast for algorithm-specific access or test introspection.
    fn as_any(&self) -> &dyn Any;
}

// This macro automatically generates the implementation of `Clone` for `Box<dyn Measurement>`.
dyn_clone::clone_trait_object!(Measurement);

pub mod accelerometer;
pub mod gps;
pub mod gyroscope;
pub mod magnetometer;
