use std::any::Any;

use crate::{
    frames::{FrameAwareState, FrameId, StateVariable},
    prelude::{Measurement, MeasurementData, MeasurementMessage},
    types::{FrameHandle, TfProvider},
};
use nalgebra::{DMatrix, DVector, Vector3};

/// A measurement model for a 3-axis magnetometer.
///
/// This model relates a measured magnetic field vector to the filter's
/// orientation state, providing an absolute heading reference.
#[derive(Debug, Clone)]
pub struct MagnetometerModel {
    /// A handle to the agent this sensor belongs to (the "body" frame).
    pub agent_handle: FrameHandle,
    /// A handle to the sensor itself (the "sensor" frame).
    pub sensor_handle: FrameHandle,
    /// The 3x3 measurement noise covariance matrix, R.
    pub r_matrix: DMatrix<f64>,
    /// The "true" magnetic field vector in the world (ENU) frame.
    /// For simulation, we define this as a constant pointing North (+Y).
    pub world_magnetic_field: Vector3<f64>,
}

impl Measurement for MagnetometerModel {
    /// Describes the semantic layout of the measurement vector `z`.
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        let sensor_frame = FrameId::Sensor(self.sensor_handle);
        vec![
            StateVariable::MagX(sensor_frame.clone()),
            StateVariable::MagY(sensor_frame.clone()),
            StateVariable::MagZ(sensor_frame.clone()),
        ]
    }

    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        _tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // This model only cares about Magnetometer data.
        if !matches!(&message.data, MeasurementData::Magnetometer(_)) {
            return None;
        }

        // Get the filter's current belief about its orientation.
        let orientation_body_to_world = filter_state.get_orientation().unwrap_or_default();

        // --- The Core Logic ---
        // Predict what the magnetometer should see by taking the true world
        // magnetic field and rotating it into the robot's body frame.
        let q_body_from_world = orientation_body_to_world.inverse();
        let predicted_mag_body = q_body_from_world * self.world_magnetic_field;

        Some(DVector::from_row_slice(predicted_mag_body.as_slice()))
    }

    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        tf: &dyn TfProvider,
    ) -> DMatrix<f64> {
        let state_dim = filter_state.dim();
        let measurement_dim = 3;
        let mut h_jac = DMatrix::zeros(measurement_dim, state_dim);
        let epsilon = 1e-8;

        let dummy_message = MeasurementMessage {
            agent_handle: self.agent_handle,
            sensor_handle: self.sensor_handle,
            timestamp: 0.0,
            data: MeasurementData::Magnetometer(Default::default()),
        };

        let z_base = match self.predict_measurement(filter_state, &dummy_message, tf) {
            Some(z) => z,
            None => return h_jac,
        };

        for j in 0..state_dim {
            let mut perturbed_state = filter_state.clone();
            perturbed_state.vector[j] += epsilon;

            // Re-normalize if we're perturbing the quaternion part (indices 6-9).
            if j >= 6 && j <= 9 {
                let mut quat_part = perturbed_state.vector.fixed_rows_mut::<4>(6);
                let norm = quat_part.norm();
                if norm > 1e-9 {
                    quat_part /= norm;
                }
            }

            if let Some(z_perturbed) =
                self.predict_measurement(&perturbed_state, &dummy_message, tf)
            {
                let derivative_column = (z_perturbed - &z_base) / epsilon;
                h_jac.column_mut(j).copy_from(&derivative_column);
            }
        }

        h_jac
    }

    fn get_r(&self) -> &DMatrix<f64> {
        &self.r_matrix
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}
