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

use crate::estimation::measurement::MeasurementModel;
use crate::prelude::MonotonicTime;
use crate::spatial::FrameAwareState;
use crate::{estimation::measurement::Unavailable, spatial::tf::TfProvider};

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
///    Returns a [`PredictOutcome`]: it skips rather than propagate on a bad step
///    or a misshapen input, and says which.
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
    /// Returns [`PredictOutcome::Applied`] when `(x, P, t)` moved, or
    /// [`PredictOutcome::Skipped`] with the [`PredictSkipReason`] when it did not:
    /// a `dt <= 0` (nothing to propagate), a control vector whose length
    /// disagrees with the dynamics' input schema, or (sigma-point filters) a
    /// `P` that cannot be factored. A skip leaves `(x, P, t)` untouched. A misshapen input is never padded with zeros: integrating the
    /// wrong quantity, or none, is a silent model error, so it is reported.
    fn predict(&mut self, dt: f64, inputs: &EstimatorInputs) -> PredictOutcome;

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

    /// Declares the current mean and covariance valid at `t`, moving neither.
    ///
    /// This is not a predict: no time is integrated, so `t` may be earlier or
    /// later than the current valid-at time. It exists for the caller that
    /// builds the prior before it can read its clock: the prior is built
    /// valid at an arbitrary time, and on its first tick the caller stamps it
    /// with the clock's time, after which every [`predict`](Self::predict)
    /// advances it by `dt`.
    fn set_valid_at(&mut self, t: MonotonicTime);

    /// Current best state estimate `(x, P, t)`.
    fn state(&self) -> &FrameAwareState;
}

/// What one [`GaussianStateEstimator::predict`] call did.
///
/// The predict-side twin of [`UpdateOutcome`], with its own reasons: a predict
/// has no measurement, so none of [`SkipReason`]'s variants apply to it.
#[derive(Debug, Clone, PartialEq)]
pub enum PredictOutcome {
    /// The state, covariance and valid-at time moved forward by `dt`.
    Applied,
    /// The predict was skipped without touching the estimate; the reason says why.
    Skipped(PredictSkipReason),
}

/// Why a [`GaussianStateEstimator::predict`] declined to propagate — split so the
/// caller can log the faults and ignore the expected cases.
///
/// Non-exhaustive: a new reason (a non-finite input, say) must not break a
/// caller's `match`. A caller outside this crate treats an unknown reason as
/// loud.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum PredictSkipReason {
    /// **Quiet.** `dt <= 0`: there is no interval to propagate over, as on the
    /// first tick.
    NonPositiveDt,
    /// **Loud.** The control vector's length is not the dynamics' input-schema
    /// length — a wiring bug the build-time input agreement check should have
    /// caught. Carries both lengths for the log.
    InputShapeMismatch { expected: usize, supplied: usize },
    /// **Loud.** `P` failed Cholesky, so a sigma-point filter cannot spread its
    /// points: the covariance is corrupt. An EKF never factors `P` to predict
    /// and does not report this.
    CovarianceNotPositiveDefinite,
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
    /// The measurement was fused; the estimate moved. Carries how surprising the
    /// measurement was against the filter's prediction.
    Applied(Innovation),
    /// The update was skipped without touching the estimate; the reason says why.
    Skipped(SkipReason),
}

/// How surprising one fused measurement was, measured against the filter's own
/// predicted spread for it.
///
/// `nis` is the normalized innovation squared, `yᵀS⁻¹y`, with `y = z − ẑ` the
/// innovation and `S` its predicted covariance. Unit-free, so sensors of
/// different kinds compare on one scale. When the filter's `S` is honest, `nis`
/// is chi-squared with `dof` degrees of freedom: its average over many updates
/// sits near `dof`. Well above means the filter is overconfident (or the
/// measurement is an outlier); well below means it is too cautious. One sample
/// is noisy; judge consistency over a window.
///
/// Gaussian-specific: it presumes a single predicted `S`. Non-exhaustive, so the
/// innovation vector, `S`, or the predictive log-likelihood can be added without
/// breaking a caller; build one with [`Innovation::new`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct Innovation {
    /// Normalized innovation squared, `yᵀS⁻¹y`.
    pub nis: f64,
    /// Degrees of freedom: the measurement's length.
    pub dof: usize,
}

impl Innovation {
    /// An innovation with normalized squared size `nis` over `dof` degrees of
    /// freedom.
    pub fn new(nis: f64, dof: usize) -> Self {
        Self { nis, dof }
    }
}

/// Why a [`GaussianStateEstimator::update`] declined to fuse a measurement —
/// split so the caller can log the faults and ignore the expected cases.
///
/// Non-exhaustive: a new reason (an outlier rejected by an innovation gate, say)
/// must not break a caller's `match`. A caller outside this crate treats an
/// unknown reason as loud.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
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
