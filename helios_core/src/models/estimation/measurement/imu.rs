use nalgebra::{DMatrix, DVector, Vector3};
use std::any::Any;
use std::fmt::Debug;

use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::messages::{MeasurementData, MeasurementMessage};
use crate::models::estimation::measurement::Measurement;
use crate::types::{FrameHandle, TfProvider};

/// A high-fidelity model for a 6-DOF IMU when used as a MEASUREMENT source.
///
/// This model is typically used with simpler dynamics models (like ConstantAcceleration)
/// where the IMU provides corrective information about the agent's acceleration and rotation.
#[derive(Debug, Clone)]
pub struct Imu6DofModel {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub r_matrix: DMatrix<f64>, // 6x6 measurement noise covariance
    pub gravity_magnitude: f64,
}

impl Measurement for Imu6DofModel {
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        let sensor_frame = FrameId::Sensor(self.sensor_handle);
        vec![
            StateVariable::Ax(sensor_frame.clone()),
            StateVariable::Ay(sensor_frame.clone()),
            StateVariable::Az(sensor_frame.clone()),
            StateVariable::Wx(sensor_frame.clone()),
            StateVariable::Wy(sensor_frame.clone()),
            StateVariable::Wz(sensor_frame.clone()),
        ]
    }

    /// Predicts the 6-element measurement vector [ax, ay, az, wx, wy, wz].
    /// It returns `Some` only if the input message contains `Imu6Dof` data.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // --- 1. Check if this model can handle the data ---
        // This model only processes `Imu6Dof` data. It ignores everything else.
        if !matches!(&message.data, MeasurementData::Imu6Dof(_)) {
            return None;
        }

        // --- At this point, we know we should proceed. ---
        let mut z_pred = DVector::zeros(6);
        let body_frame = FrameId::Body(self.agent_handle);

        // --- 2. Get Transform Data ---
        let tf_sensor_from_body = tf
            .get_transform(self.agent_handle, self.sensor_handle)
            .unwrap_or_default();
        let r_body_to_sensor = tf_sensor_from_body.translation.vector;
        let rot_sensor_from_body = tf_sensor_from_body.rotation;

        // --- 3. Extract States from Filter ---
        // Get the filter's current belief about its own state.
        let linear_accel_body = filter_state
            .get_vector3(&StateVariable::Ax(body_frame.clone()))
            .unwrap_or_default();
        let angular_vel_body = filter_state
            .get_vector3(&StateVariable::Wx(body_frame.clone()))
            .unwrap_or_default();
        let angular_accel_body = filter_state
            .get_vector3(&StateVariable::Alphax(body_frame.clone()))
            .unwrap_or_default();
        let orientation_body_to_world = filter_state.get_orientation().unwrap_or_default();

        // --- 4. Predict Angular Velocity (Gyroscope part) ---
        let predicted_gyro = rot_sensor_from_body.inverse() * angular_vel_body;
        z_pred.fixed_rows_mut::<3>(3).copy_from(&predicted_gyro);

        // --- 5. Predict Linear Acceleration (Accelerometer part) ---
        let tangential_accel = angular_accel_body.cross(&r_body_to_sensor);
        let centripetal_accel = angular_vel_body.cross(&angular_vel_body.cross(&r_body_to_sensor));
        let total_kinematic_accel_at_sensor =
            linear_accel_body + tangential_accel + centripetal_accel;

        let q_body_from_world = orientation_body_to_world.inverse();
        let gravity_world = Vector3::new(0.0, 0.0, -self.gravity_magnitude);
        let gravity_effect_in_body = q_body_from_world * gravity_world;

        let proper_accel_in_body_frame = total_kinematic_accel_at_sensor - gravity_effect_in_body;
        let predicted_accel = rot_sensor_from_body.inverse() * proper_accel_in_body_frame;
        z_pred.fixed_rows_mut::<3>(0).copy_from(&predicted_accel);

        // Since we successfully processed the data, return the prediction.
        Some(z_pred)
    }

    /// Calculates the Jacobian H using numerical differentiation.
    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        tf: &dyn TfProvider,
    ) -> DMatrix<f64> {
        // This numerical implementation is robust and works well here.
        let state_dim = filter_state.dim();
        let mut h_jac = DMatrix::zeros(6, state_dim);
        let epsilon = 1e-8;

        // We need a dummy message to pass to predict_measurement.
        // The data variant just needs to be the one this model accepts.
        let dummy_message = MeasurementMessage {
            agent_handle: self.agent_handle,
            sensor_handle: self.sensor_handle,
            timestamp: 0.0,
            data: MeasurementData::Imu6Dof(Default::default()),
        };

        // The baseline prediction must be valid for the subtraction to work.
        let z_base = match self.predict_measurement(filter_state, &dummy_message, tf) {
            Some(z) => z,
            None => return h_jac, // Should not happen if called correctly
        };

        for j in 0..state_dim {
            let mut perturbed_state = filter_state.clone();
            perturbed_state.vector[j] += epsilon;

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
