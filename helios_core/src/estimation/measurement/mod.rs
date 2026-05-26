//! Measurement model trait and concrete sensor implementations.
//!
//! Each sensor type implements [`MeasurementModel`], which describes the function
//! `z = h(x) + v` — the ideal measurement predicted from filter state, plus noise.
//!
//! # Implementing a New Sensor Model
//!
//! 1. Create `estimation/measurement/my_sensor.rs`, implement [`MeasurementModel`].
//! 2. `predict_measurement` returns `None` when the model cannot produce a
//!    prediction for this state (e.g. a required TF is missing). Never panic.
//! 3. The default [`MeasurementModel::jacobian`] computes `H` via finite differences
//!    on `predict_measurement`. Override only when an analytic Jacobian is faster
//!    or more accurate.
//! 4. Re-export from this `mod.rs`.

use crate::frames::FrameAwareState;
use crate::ports::TfProvider;
use nalgebra::{DMatrix, DVector};

/// Mathematical model of a sensor: `z = h(x) + v`.
///
/// Describes the deterministic part of a sensor — the function that maps filter
/// state to an ideal measurement. Noise covariance `R` is **not** part of the
/// model; it lives at the call site (handler / standalone caller), is constructed
/// per physical sensor, and is passed in per `update`. This split lets one model
/// serve N sensors of differing quality and lets adaptive callers vary `R` per
/// reading without mutating the model.
pub trait MeasurementModel: Send + Sync {
    /// Computes the ideal predicted measurement `z_pred = h(x)` from the filter state.
    ///
    /// Used during the EKF/UKF update to form the innovation `y = z - z_pred`.
    /// `tf` is `None` when no transform tree is available; models that need TF for
    /// a frame conversion should return `None` in that case and the filter will
    /// silently skip the update.
    fn predict_measurement(
        &self,
        state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
    ) -> Option<DVector<f64>>;

    /// Measurement Jacobian `H = ∂h/∂x` of shape `(dim(), state.dim())`.
    ///
    /// Default impl computes `H` via central finite differences on
    /// [`predict_measurement`] using adaptive epsilon `ε = 1e-5 · (1 + |xᵢ|)`.
    /// Override for analytic Jacobians where performance or accuracy matters.
    fn jacobian(&self, state: &FrameAwareState, tf: Option<&dyn TfProvider>) -> DMatrix<f64> {
        let m = self.dim();
        let n = state.dim();
        let mut h = DMatrix::zeros(m, n);
        let Some(z_base) = self.predict_measurement(state, tf) else {
            return h;
        };
        if z_base.nrows() != m {
            return h;
        }
        for j in 0..n {
            let eps = 1e-5 * (1.0 + state.state.vector[j].abs());
            let mut perturbed = state.clone();
            perturbed.state.vector[j] += eps;
            if let Some(z_pert) = self.predict_measurement(&perturbed, tf) {
                if z_pert.nrows() == m {
                    let col = (z_pert - &z_base) / eps;
                    h.column_mut(j).copy_from(&col);
                }
            }
        }
        h
    }

    /// Dimension of the measurement vector `z`.
    fn dim(&self) -> usize;
}

pub mod accelerometer;
pub mod gps;
pub mod gyroscope;
pub mod magnetometer;
