// helios_core/src/models/imu.rs

use crate::abstractions::{Measurement, TfProvider};
use crate::frames::{FrameAwareState, FrameHandle, FrameId, StateVariable};
use nalgebra::{DMatrix, DVector, Rotation3, Vector3};

// --- Concrete Model for a 6-DOF IMU ---
#[derive(Debug)]
pub struct Imu6DofModel {
    /// A handle to the parent agent this sensor belongs to.
    pub agent_handle: FrameHandle,
    /// A handle to the sensor itself.
    pub sensor_handle: FrameHandle,
    /// The 6x6 measurement noise covariance matrix `R`.
    pub r_matrix: DMatrix<f64>,
}

impl Measurement for Imu6DofModel {
    /// Describes the raw output of this sensor in its own native frame.
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

    fn get_r(&self) -> &DMatrix<f64> {
        &self.r_matrix
    }

    /// Predicts the measurement based on the filter's state and the kinematic context.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        context: &dyn TfProvider,
    ) -> DVector<f64> {
        let mut z_pred = DVector::zeros(6);
        let body_handle = self.agent_handle;

        // Get the rotation from the body frame to this sensor's frame.
        let rot_sensor_from_body = context
            .get_transform(body_handle, self.sensor_handle)
            .map(|t| t.rotation)
            .unwrap_or_default();

        // Find the estimated body-frame acceleration in the state vector.
        if let Some(idx) = filter_state.find_idx(&StateVariable::Ax(FrameId::Body(body_handle))) {
            let accel_body = filter_state.vector.fixed_rows::<3>(idx);
            // Rotate the estimated acceleration into the sensor's frame for prediction.
            z_pred
                .fixed_rows_mut::<3>(0)
                .copy_from(&(rot_sensor_from_body * accel_body));
        }

        // Find the estimated body-frame angular velocity in the state vector.
        if let Some(idx) = filter_state.find_idx(&StateVariable::Wx(FrameId::Body(body_handle))) {
            let ang_vel_body = filter_state.vector.fixed_rows::<3>(idx);
            // Rotate the estimated angular velocity into the sensor's frame.
            z_pred
                .fixed_rows_mut::<3>(3)
                .copy_from(&(rot_sensor_from_body * ang_vel_body));
        }

        z_pred
    }

    /// Calculates the Jacobian H, which maps a change in the state to a change in the measurement.
    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        context: &dyn TfProvider,
    ) -> DMatrix<f64> {
        let mut h = DMatrix::zeros(6, filter_state.dim());
        let body_handle = self.agent_handle;

        // Get the rotation from the body frame to this sensor's frame.
        if let Some(rot_sensor_from_body) = context
            .get_transform(body_handle, self.sensor_handle)
            .map(|t| t.rotation)
        {
            // --- THE FIX ---
            // Convert the `Rotation3` type into a standard `Matrix3`.
            let rot_matrix = *rot_sensor_from_body.to_rotation_matrix().matrix();

            // Find the corresponding states in the EKF layout and insert the rotation matrix.
            if let Some(idx) = filter_state.find_idx(&StateVariable::Ax(FrameId::Body(body_handle)))
            {
                // Now `copy_from` receives the type it expects.
                h.fixed_view_mut::<3, 3>(0, idx).copy_from(&rot_matrix);
            }
            if let Some(idx) = filter_state.find_idx(&StateVariable::Wx(FrameId::Body(body_handle)))
            {
                h.fixed_view_mut::<3, 3>(3, idx).copy_from(&rot_matrix);
            }
        }

        h
    }
}

// --- Concrete Model for a 9-DOF IMU (example structure) ---
// #[derive(Debug)]
// pub struct Imu9DofModel {
//     pub agent_handle: FrameHandle,
//     pub sensor_handle: FrameHandle,
//     pub r_matrix: DMatrix<f64>, // 9x9
//     // It might need extra contextual data
//     pub world_magnetic_field: Vector3<f64>,
// }

// impl Measurement for Imu9DofModel {
//     // Its layout would have 9 variables...
//     // Its `predict_measurement` and `calculate_jacobian` would be more complex,
//     // involving the rotation of the world_magnetic_field vector.
//     // ...
// }
