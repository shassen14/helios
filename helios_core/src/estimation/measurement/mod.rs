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

use crate::data::{ports::TfProvider, MonotonicTime};
use crate::frames::FrameAwareState;
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
        at: MonotonicTime,
    ) -> Option<DVector<f64>>;

    /// Measurement Jacobian `H = ∂h/∂δx` of shape `(dim(), tangent_dim)`.
    ///
    /// Columns live in **tangent** (error) space, not stored-component space, so
    /// `H` lines up with the covariance `P` and the transition `F` the filter
    /// carries. The two differ wherever a block stores more numbers than it has
    /// degrees of freedom — an orientation block is 4 stored, 3 tangent — so a
    /// per-storage-component perturbation would be both mis-sized and off the
    /// manifold.
    ///
    /// Default impl finite-differences [`predict_measurement`]: each column `j`
    /// nudges the `j`-th tangent coordinate by `ε` and retracts onto the manifold
    /// with `oplus`, then differences the prediction. One step size scales the
    /// whole Jacobian, `ε = 1e-5 · (1 + ‖x‖∞)` — the tangent-space analog of the
    /// crate's adaptive rule, since a tangent index has no single stored
    /// component to scale against. Override for analytic Jacobians where
    /// performance or accuracy matters; an override returns the same shape.
    fn jacobian(
        &self,
        state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
        at: MonotonicTime,
    ) -> DMatrix<f64> {
        // Rows = measurement length; columns = tangent (error) DOF, not stored
        // components — H must match P and F, which are tangent-indexed.
        let m = self.dim();
        let n = state.tangent_dim();
        let mut h = DMatrix::zeros(m, n);

        let Some(z_base) = self.predict_measurement(state, tf, at) else {
            return h;
        };

        if z_base.nrows() != m {
            return h;
        }

        let eps = 1e-5 * (1.0 + state.mean.amax());
        for j in 0..n {
            // Bump the j-th tangent coordinate and retract onto the manifold, so
            // an orientation perturbation rotates the quaternion instead of
            // pushing it off the unit sphere.
            let mut delta = DVector::zeros(n);
            delta[j] = eps;
            let mut perturbed = state.clone();
            perturbed.oplus_assign(&delta);

            if let Some(z_pert) = self.predict_measurement(&perturbed, tf, at) {
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
