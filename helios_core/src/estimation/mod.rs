//! Gaussian-family state estimation traits and context types.
//!
//! Defines [`GaussianStateEstimator`], the trait shared by EKF, UKF, ESKF, and
//! information-form filters. Concrete implementations live in [`filters`].
use nalgebra::{DMatrix, DVector};

use crate::data::primitives::TfProvider;
use crate::estimation::measurement::MeasurementModel;
use crate::frames::FrameAwareState;

/// World-context passed into mappers and other modules that still consume
/// `ModuleInput::Measurement`.
#[derive(Default)]
pub struct FilterContext<'a> {
    pub tf: Option<&'a dyn TfProvider>,
}

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
    ///   Must be square with side equal to `model.dim()`.
    /// * `tf` — transform tree access; `None` is valid and is forwarded to the
    ///   model, which may return `None` from `predict_measurement` to signal that
    ///   the update cannot proceed. The filter silently skips in that case.
    fn update(
        &mut self,
        z: &DVector<f64>,
        model: &dyn MeasurementModel,
        r: &DMatrix<f64>,
        tf: Option<&dyn TfProvider>,
    );

    /// Current best state estimate `(x, P, t)`.
    fn state(&self) -> &FrameAwareState;
}

pub mod dynamics;
pub mod filters;
pub mod measurement;
