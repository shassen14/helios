// helios_core/src/models/measurement/imu.rs

use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::prelude::Measurement;
use crate::types::{FrameHandle, TfProvider};
use nalgebra::{DMatrix, DVector, Isometry3, Vector3};

// --- Concrete Model for a 6-DOF IMU ---
#[derive(Debug, Clone)]
pub struct Imu6DofModel {
    /// A handle to the parent agent this sensor belongs to.
    pub agent_handle: FrameHandle,
    /// A handle to the sensor itself.
    pub sensor_handle: FrameHandle,
    /// The 6x6 measurement noise covariance matrix `R`.
    pub r_matrix: DMatrix<f64>,
    /// Gravity magnitude to calculate the proper acceleration
    pub gravity_magnitude: f64,
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

    // /// Predicts the measurement based on the filter's state and the kinematic context.
    // fn predict_measurement(
    //     &self,
    //     filter_state: &FrameAwareState,
    //     tf: &dyn TfProvider, // We now need the transform provider
    // ) -> DVector<f64> {
    //     let mut z_pred = DVector::zeros(6);

    //     // --- Get Handles for Body and Sensor ---
    //     // This assumes the model is constructed with the correct handles.
    //     let agent_handle = self.agent_handle;
    //     let sensor_handle = self.sensor_handle;
    //     let agent_frame = FrameId::Body(self.agent_handle);

    //     // --- 1. Get the Transform from Body to IMU ---
    //     // This gives us the lever arm vector and the relative rotation.
    //     let tf_sensor_from_body = tf
    //         .get_transform(agent_handle, sensor_handle)
    //         .unwrap_or_else(Isometry3::identity); // Default to identity if TF fails

    //     let r_body_to_sensor_in_body_frame = tf_sensor_from_body.translation.vector;
    //     let rot_sensor_from_body = tf_sensor_from_body.rotation;

    //     // --- 2. Get Required States from the Filter ---
    //     // These are all expressed in the BODY frame.
    //     let linear_accel_body = filter_state
    //         .get_vector3(&StateVariable::Ax(agent_frame.clone()))
    //         .unwrap_or_default();
    //     let angular_vel_body = filter_state
    //         .get_vector3(&StateVariable::Wx(agent_frame.clone()))
    //         .unwrap_or_default();
    //     let angular_accel_body = filter_state
    //         .get_vector3(&StateVariable::Alphax(agent_frame.clone()))
    //         .unwrap_or_default();

    //     // ======================================================================
    //     // == PREDICT ANGULAR VELOCITY ==
    //     // ======================================================================
    //     // The gyroscope measures the body's angular velocity, but expressed
    //     // in the IMU's own coordinate frame. We just need to rotate it.
    //     let predicted_gyro = rot_sensor_from_body.inverse() * angular_vel_body;
    //     z_pred.fixed_rows_mut::<3>(3).copy_from(&predicted_gyro);

    //     // ======================================================================
    //     // == PREDICT LINEAR ACCELERATION (with all effects) ==
    //     // ======================================================================

    //     // --- 2a. Calculate Lever Arm Accelerations (in the BODY frame) ---
    //     let tangential_accel = angular_accel_body.cross(&r_body_to_sensor_in_body_frame);
    //     let centripetal_accel =
    //         angular_vel_body.cross(&angular_vel_body.cross(&r_body_to_sensor_in_body_frame));

    //     // --- 2b. Start with the body's coordinate acceleration ---
    //     let mut total_accel_at_sensor_in_body_frame = linear_accel_body;

    //     // --- 2c. Add the lever arm effects ---
    //     total_accel_at_sensor_in_body_frame += tangential_accel;
    //     total_accel_at_sensor_in_body_frame += centripetal_accel;

    //     // --- 2d. Subtract Gravity ---
    //     // We do this last. We need the filter's estimate of the body's orientation
    //     // relative to the world.
    //     if let Some(orientation_body_to_world) = filter_state.get_orientation() {
    //         let q_body_from_world = orientation_body_to_world.inverse();
    //         let gravity_world = Vector3::new(0.0, 0.0, -self.gravity_magnitude);

    //         // The IMU feels the force that counteracts gravity.
    //         total_accel_at_sensor_in_body_frame -= q_body_from_world * gravity_world;
    //     }

    //     // --- 2e. Final Rotation ---
    //     // The total acceleration is now calculated, but it's expressed in the BODY frame.
    //     // We must rotate it into the SENSOR's frame to match what the real sensor would output.
    //     let predicted_accel = rot_sensor_from_body.inverse() * total_accel_at_sensor_in_body_frame;
    //     z_pred.fixed_rows_mut::<3>(0).copy_from(&predicted_accel);

    //     z_pred
    // }

    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        tf: &dyn TfProvider,
    ) -> DVector<f64> {
        let mut z_pred = DVector::zeros(6);

        // --- 1. Get Handles and Transform Data ---
        let agent_handle = self.agent_handle;
        let sensor_handle = self.sensor_handle;
        let tf_sensor_from_body = tf
            .get_transform(agent_handle, sensor_handle)
            .unwrap_or_default();
        let r_body_to_sensor = tf_sensor_from_body.translation.vector;
        let rot_sensor_from_body = tf_sensor_from_body.rotation;

        // --- 2. Extract State Variables from the Filter ---
        let body_frame = FrameId::Body(agent_handle);
        let linear_accel_body = filter_state
            .get_vector3(&StateVariable::Ax(body_frame.clone()))
            .unwrap_or_default();
        let angular_vel_body = filter_state
            .get_vector3(&StateVariable::Wx(body_frame.clone()))
            .unwrap_or_default();
        let angular_accel_body = filter_state
            .get_vector3(&StateVariable::Alphax(body_frame.clone()))
            .unwrap_or_default();

        // This is the filter's belief about its orientation.
        let orientation_body_to_world = filter_state.get_orientation().unwrap_or_default();

        // --- 3. Predict Angular Velocity (Gyroscope) ---
        // This part is simple: just rotate the body's angular velocity into the sensor's frame.
        let predicted_gyro = rot_sensor_from_body.inverse() * angular_vel_body;
        z_pred.fixed_rows_mut::<3>(3).copy_from(&predicted_gyro);

        // --- 4. Predict Linear Acceleration (Accelerometer) ---

        // 4a. Calculate kinematic accelerations due to rotation (lever arm effect).
        // These are calculated in the body frame.
        let tangential_accel = angular_accel_body.cross(&r_body_to_sensor);
        let centripetal_accel = angular_vel_body.cross(&angular_vel_body.cross(&r_body_to_sensor));

        // 4b. Combine all *coordinate* accelerations in the body frame.
        let total_coordinate_accel_at_sensor_in_body =
            linear_accel_body + tangential_accel + centripetal_accel;

        // 4c. Calculate the gravity vector as seen by the sensor.
        // This is the core of the fix.
        // The gravity vector in the world (ENU) frame is constant.
        let gravity_world = Vector3::new(0.0, 0.0, -self.gravity_magnitude);

        // We need to rotate this world-frame vector into the sensor's local frame to subtract it.
        // We can do this in one step: R_sensor_from_world = R_sensor_from_body * R_body_from_world
        let rot_body_from_world = orientation_body_to_world.inverse();
        let rot_sensor_from_world = rot_sensor_from_body.inverse() * rot_body_from_world;
        let gravity_in_sensor_frame = rot_sensor_from_world * gravity_world;

        // 4d. The final predicted accelerometer output is the combination of all effects.
        // First, rotate the coordinate acceleration into the sensor's frame.
        let coordinate_accel_in_sensor_frame =
            rot_sensor_from_body.inverse() * total_coordinate_accel_at_sensor_in_body;

        // Then, subtract the gravity effect (which is already in the sensor frame).
        // This directly models: PredictedOutput = (TotalCoordAccel) - (Gravity)
        let predicted_accel_sensor_frame =
            coordinate_accel_in_sensor_frame - gravity_in_sensor_frame;

        z_pred
            .fixed_rows_mut::<3>(0)
            .copy_from(&predicted_accel_sensor_frame);

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
