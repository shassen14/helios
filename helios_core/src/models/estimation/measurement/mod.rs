// helios_core/src/models/measurement/mod.rs

use crate::frames::{FrameAwareState, StateVariable};
use crate::prelude::MeasurementMessage;
use crate::types::TfProvider;
use dyn_clone::DynClone;
use nalgebra::{DMatrix, DVector};
use std::any::Any;
use std::fmt::Debug;

// --- MEASUREMENT MODEL TRAIT ---
// Represents the mathematical model of a sensor. `z = h(x) + v`
pub trait Measurement: DynClone + Debug + Send + Sync {
    /// Describes the layout of the measurement vector `z` in the SENSOR'S NATIVE FRAME.
    fn get_measurement_layout(&self) -> Vec<StateVariable>;

    /// Returns the measurement noise covariance matrix `R`.
    fn get_r(&self) -> &DMatrix<f64>;

    /// **Predicts the ideal measurement `z_pred = h(x)` from the filter's state.**
    ///
    /// This is the key change. If this model can process the data in the
    /// provided message, it returns `Some(DVector)`. If it cannot or should
    //  ignore it, it returns `None`.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        tf: &dyn TfProvider,
    ) -> Option<DVector<f64>>;

    /// Calculates the measurement Jacobian `H = ∂h/∂x`.
    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        context: &dyn TfProvider,
    ) -> DMatrix<f64>;

    fn as_any(&self) -> &dyn Any;
}

// This macro automatically generates the implementation of `Clone` for `Box<dyn Measurement>`.
dyn_clone::clone_trait_object!(Measurement);

pub mod gps;
pub mod imu;
pub mod magnetometer;
