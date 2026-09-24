//! Gaussian-family state estimation traits and context types.
//!
//! Defines [`GaussianStateEstimator`], the trait shared by EKF, UKF, ESKF, and
//! information-form filters. Concrete implementations live in [`filters`].

pub mod augmentation;
pub mod carrier;
pub mod dynamics;
pub mod filters;
pub mod measurement;
pub mod schema;

use crate::prelude::MonotonicTime;
use crate::estimation::measurement::MeasurementModel;
use crate::spatial::FrameAwareState;
use crate::{spatial::tf::TfProvider, estimation::measurement::Unavailable};

use nalgebra::{DMatrix, DVector};

/// Predict-side inputs for a Gaussian estimator.
///
/// Wrapper struct (instead of a bare `DVector<f64>`) so additional fields can be
/// added without churning the trait signature.
pub struct EstimatorInputs {
    pub control: DVector<f64>,
}

/// Contract for any Gaussian-family state estimator.
///
/// State is a Gaussian distribution `(x, P)` (or its information-form dual). The
/// estimator exposes:
///
/// 1. **`predict`** — propagates `(x, P)` forward using dynamics + control input.
/// 2. **`update`** — fuses one measurement using a [`MeasurementModel`] (the math)
///    and a noise covariance `R` (the per-sensor noise). The model does not hold
///    `R`; it is supplied per call so one model can serve sensors of differing
///    quality and so callers can vary `R` adaptively without mutating the model.
///
/// # Implementing a New Filter
///
/// 1. Create `estimation/filters/my_filter.rs` and implement this trait.
/// 2. Re-export from `estimation/filters/mod.rs`.
/// 3. The pipeline node wrapper (`GaussianEstimatorNode` in `helios_runtime`) will
///    consume any `Box<dyn GaussianStateEstimator>` without further changes.
///
/// Implementations must be `Send + Sync`. They use `&mut self` for predict/update;
/// interior mutability inside the filter struct is forbidden
pub trait GaussianStateEstimator: Send + Sync {
    /// Propagates the state and covariance forward by `dt` seconds.
    ///
    /// On `dt <= 0` the call must be a no-op.
    fn predict(&mut self, dt: f64, inputs: &EstimatorInputs);

    /// Fuses one measurement to correct the current estimate.
    ///
    /// * `z` — measurement vector. Caller is responsible for producing this from
    ///   a typed `SensorReading<T>` via `T::to_measurement_vector()`.
    /// * `model` — the sensor's mathematical model. Provides `h(x)` and `H`.
    /// * `r` — measurement noise covariance for this specific sensor and reading.
    ///   Must be square with side equal to the measurement length (`z.nrows()`).
    /// * `tf` — transform tree access; `None` is valid and is forwarded to the
    ///   model, which may decline to predict. The filter never fuses a
    ///   half-formed correction — it skips the update whenever the model declines
    ///   or a numerical guard trips.
    ///
    /// Returns an [`UpdateOutcome`]: [`UpdateOutcome::Applied`] when the estimate
    /// was corrected, or [`UpdateOutcome::Skipped`] carrying the [`SkipReason`]
    /// otherwise. The reason lets the caller distinguish an expected shortfall
    /// (cold start, no provider) from a fault (an unresolved transform, a
    /// non-positive-definite covariance) and log only the latter. This core never
    /// logs; classification is its whole contribution and emission is the
    /// runtime caller's.
    fn update(
        &mut self,
        z: &DVector<f64>,
        model: &dyn MeasurementModel,
        r: &DMatrix<f64>,
        tf: Option<&dyn TfProvider>,
        at: MonotonicTime,
    ) -> UpdateOutcome;

    /// Current best state estimate `(x, P, t)`.
    fn state(&self) -> &FrameAwareState;
}

/// What one [`GaussianStateEstimator::update`] call did.
///
/// Replaces a bare `()` return, which conflated "the estimate was corrected"
/// with "the measurement was dropped" — the silence that let a broken transform
/// leave the filter running unaided with no trace. Every skip now carries a
/// [`SkipReason`], so the runtime caller can surface the faults and stay quiet
/// on the expected shortfalls.
#[derive(Debug, Clone, PartialEq)]
pub enum UpdateOutcome {
    /// The measurement was fused; the estimate moved.
    Applied,
    /// The update was skipped without touching the estimate; the reason says why.
    Skipped(SkipReason),
}

/// Why a [`GaussianStateEstimator::update`] declined to fuse a measurement —
/// split so the caller can log the faults and ignore the expected cases.
#[derive(Debug, Clone, PartialEq)]
pub enum SkipReason {
    /// The measurement model declined to predict. Carries the model's own
    /// [`Unavailable`] reason, which itself classifies quiet vs. loud — this
    /// variant forwards that judgment unchanged.
    Model(Unavailable),
    /// **Loud.** `R`, the incoming `z`, or the model's prediction disagreed on
    /// length — a wiring bug the build-time schema check should have caught.
    MeasurementShapeMismatch,
    /// **Loud.** The innovation covariance failed Cholesky, i.e. `P` has lost
    /// positive-definiteness. Not merely a dropped measurement — a sign the
    /// filter's covariance is corrupt.
    CovarianceNotPositiveDefinite,
}
