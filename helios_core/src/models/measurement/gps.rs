// helios_core/src/models/measurement/gps.rs

use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::models::measurement::Measurement;
use crate::types::TfProvider;
use nalgebra::{DMatrix, DVector};

#[derive(Debug, Clone)] // Make sure it's cloneable for the dyn-clone
pub struct GpsModel {
    // The R matrix for this sensor
    pub noise_covariance: DMatrix<f64>,
}

impl Measurement for GpsModel {
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Pz(FrameId::World),
        ]
    }

    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        _tf: &dyn TfProvider,
    ) -> DVector<f64> {
        // --- THIS IS THE CRITICAL LOGIC ---
        // We need to build a 3x1 vector `z_pred` from the 9x1 `filter_state`.

        let mut z_pred = DVector::zeros(3); // We are predicting a 3-element measurement.

        // Find the indices of Px, Py, Pz in the filter's state layout.
        if let Some(px_idx) = filter_state.find_idx(&StateVariable::Px(FrameId::World)) {
            z_pred[0] = filter_state.vector[px_idx];
        }
        if let Some(py_idx) = filter_state.find_idx(&StateVariable::Py(FrameId::World)) {
            z_pred[1] = filter_state.vector[py_idx];
        }
        if let Some(pz_idx) = filter_state.find_idx(&StateVariable::Pz(FrameId::World)) {
            z_pred[2] = filter_state.vector[pz_idx];
        }

        z_pred // Return the 3x1 predicted measurement.
    }

    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        _tf: &dyn TfProvider,
    ) -> DMatrix<f64> {
        // The Jacobian H must be size (measurement_dims x state_dims) -> 3x9
        let mut h_jac = DMatrix::zeros(3, filter_state.dim());

        // d(z_pred_x) / d(Px) = 1
        if let Some(px_idx) = filter_state.find_idx(&StateVariable::Px(FrameId::World)) {
            h_jac[(0, px_idx)] = 1.0;
        }
        // d(z_pred_y) / d(Py) = 1
        if let Some(py_idx) = filter_state.find_idx(&StateVariable::Py(FrameId::World)) {
            h_jac[(1, py_idx)] = 1.0;
        }
        // d(z_pred_z) / d(Pz) = 1
        if let Some(pz_idx) = filter_state.find_idx(&StateVariable::Pz(FrameId::World)) {
            h_jac[(2, pz_idx)] = 1.0;
        }

        h_jac
    }

    fn get_r(&self) -> &DMatrix<f64> {
        &self.noise_covariance
    }
}
