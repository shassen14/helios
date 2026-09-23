//! Measurement model trait and concrete sensor implementations.
//!
//! Each sensor type implements [`MeasurementModel`], which describes the function
//! `z = h(x) + v` — the ideal measurement predicted from filter state, plus noise.
//!
//! # Implementing a New Sensor Model
//!
//! 1. Create `estimation/measurement/my_sensor.rs`, implement [`MeasurementModel`].
//! 2. `predict_measurement` returns a [`Prediction`]: [`Prediction::Ready`] with
//!    the predicted `z`, or [`Prediction::Unavailable`] carrying *why* it could
//!    not predict. Classify the reason honestly — an expected shortfall
//!    ([`Unavailable::ColdStart`], [`Unavailable::NoProvider`]) is quiet, while a
//!    broken precondition ([`Unavailable::MissingTransform`],
//!    [`Unavailable::ConventionMismatch`]) is a fault the caller surfaces loudly.
//!    Never panic. See [`Prediction`] for why the distinction matters.
//! 3. The default [`MeasurementModel::jacobian`] computes `H` via finite differences
//!    on `predict_measurement`. Override only when an analytic Jacobian is faster
//!    or more accurate.
//! 4. Re-export from this `mod.rs`.

use crate::prelude::{MonotonicTime, TfProvider};
use crate::estimation::schema::MeasurementSchema;
use crate::frames::{FrameAwareState, FrameId};
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
    /// Returns [`Prediction::Ready`] on success. When the model cannot predict, it
    /// returns [`Prediction::Unavailable`] carrying the *reason* rather than a bare
    /// absence: `tf == None` and a not-yet-initialized state block are expected
    /// (quiet), while an unresolved transform or a convention mismatch are faults
    /// the caller reports. The filter still skips the update in every non-`Ready`
    /// case; the reason exists so the skip need not be silent. Never panic.
    fn predict_measurement(
        &self,
        state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
        at: MonotonicTime,
    ) -> Prediction;

    /// Measurement Jacobian `H = ∂h/∂δx` of shape `(dim(), tangent_dim)`.
    ///
    /// Columns live in **tangent** (error) space, not stored-component space, so
    /// `H` lines up with the covariance `P` and the transition `F` the filter
    /// carries. The two differ wherever a block stores more numbers than it has
    /// degrees of freedom — an orientation block is 4 stored, 3 tangent — so a
    /// per-storage-component perturbation would be both mis-sized and off the
    /// manifold.
    ///
    /// Default impl finite-differences [`Self::predict_measurement`]: each column `j`
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
        // H is (measurement length × tangent DOF): columns are tangent (error)
        // coordinates, not stored components, so H matches P and F, which are
        // tangent-indexed. The row count comes from the prediction itself — with
        // no `predict_measurement`, there are no rows, so return a zero-row H the
        // caller shape-checks away (the EKF update already bails on `None` before
        // reaching here, so this branch is for standalone callers).
        let n = state.tangent_dim();

        let Prediction::Ready(z_base) = self.predict_measurement(state, tf, at) else {
            return DMatrix::zeros(0, n);
        };

        let m = z_base.nrows();
        let mut h = DMatrix::zeros(m, n);

        let eps = 1e-5 * (1.0 + state.mean.amax());
        for j in 0..n {
            // Bump the j-th tangent coordinate and retract onto the manifold, so
            // an orientation perturbation rotates the quaternion instead of
            // pushing it off the unit sphere.
            let mut delta = DVector::zeros(n);
            delta[j] = eps;
            let mut perturbed = state.clone();
            perturbed.oplus_assign(&delta);

            if let Prediction::Ready(z_pert) = self.predict_measurement(&perturbed, tf, at) {
                if z_pert.nrows() == m {
                    let col = (z_pert - &z_base) / eps;
                    h.column_mut(j).copy_from(&col);
                }
            }
        }
        h
    }

    /// The measurement's typed shape: the ordered blocks the model predicts and
    /// the axis convention each is expressed in.
    ///
    /// The measurement analogue of a dynamics model's
    /// [`StateSchema`](crate::estimation::schema::StateSchema) — it names every
    /// component of `z` and the frame it lives in, so an estimator can verify at
    /// construction that model and state agree on conventions at the `h(x)`
    /// seam. `schema().dim()` gives the measurement length; the kernel itself
    /// sizes off the incoming `z.nrows()`, so the schema is consulted only at
    /// build time (agreement check + sizing), never per tick.
    fn schema(&self) -> MeasurementSchema;
}

/// The outcome of [`MeasurementModel::predict_measurement`].
///
/// Replaces a bare `Option<DVector>`, which conflated two opposite meanings:
/// "this model does not apply right now" (expected — fires every tick during
/// cold-start) and "I could not resolve a transform I needed" (a bug). A `warn!`
/// keyed on the `Option` would fire on both, flood the log at startup, get muted,
/// and be useless exactly when the real fault fired. Carrying the reason lets the
/// caller (the runtime aiding handler) decide loud-vs-quiet per variant.
#[derive(Debug, Clone, PartialEq)]
pub enum Prediction {
    /// `h(x)` was computed; the payload is the predicted measurement `z_pred`.
    Ready(DVector<f64>),
    /// No prediction this tick; the variant says why, and how loud that is.
    Unavailable(Unavailable),
}

/// Why a [`MeasurementModel`] declined to predict — split into expected
/// (quiet) and fault (loud) cases so the caller can log only the surprising ones.
#[derive(Debug, Clone, PartialEq)]
pub enum Unavailable {
    /// Quiet. A required state block is not yet initialized (normal startup).
    ColdStart,
    /// Quiet. No transform provider at all — the standalone / test path; in the
    /// pipeline `tf` is always present.
    NoProvider,
    /// **Loud.** A provider was present but the named edge did not resolve — the
    /// recurring silent-aiding-drop bug class (broken tf graph, mislabelled
    /// frame). `from`/`to` name the edge that was queried.
    MissingTransform { from: FrameId, to: FrameId },
    /// **Loud.** The edge resolved but its axis conventions disagreed with what
    /// the model expects — a calibration / convention-declaration bug. `from`/`to`
    /// name the edge.
    ConventionMismatch { from: FrameId, to: FrameId },
}

pub mod accelerometer;
pub mod gps;
pub mod gyroscope;
pub mod magnetometer;
