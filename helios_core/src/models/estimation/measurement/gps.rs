// helios_core/src/models/measurement/gps.rs

use std::any::Any;

use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::models::estimation::measurement::Measurement;
use crate::prelude::{MeasurementData, MeasurementMessage};
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
        message: &MeasurementMessage,
        _tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // This model only cares about GPS data.
        if let MeasurementData::GpsPosition(_) = &message.data {
            // It's the right data type, so we proceed.
            // Predict the position from the state vector.
            if let Some(pos_vec) = filter_state.get_vector3(&StateVariable::Px(FrameId::World)) {
                Some(DVector::from_row_slice(pos_vec.as_slice()))
            } else {
                None // The state vector doesn't have position, can't predict.
            }
        } else {
            // It's not GPS data, so this model ignores it.
            None
        }
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

    fn as_any(&self) -> &dyn Any {
        self
    }
}
