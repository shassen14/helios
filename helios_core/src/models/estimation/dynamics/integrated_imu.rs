use crate::models::estimation::dynamics::EstimationDynamics;
use crate::prelude::MeasurementData;
use crate::types::{Control, FrameHandle, State};
use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};

/// A dynamics model that integrates raw IMU measurements (as control inputs)
/// to propagate a 15-state inertial navigation system (INS) state vector.
///
/// This model correctly handles 3D rotations and estimates IMU biases.
#[derive(Debug, Clone)]
pub struct IntegratedImuModel {
    /// The handle of the agent this model describes.
    pub agent_handle: FrameHandle,
    /// The magnitude of gravity, used to counteract its effect during prediction.
    pub gravity_magnitude: f64,
}

impl EstimationDynamics for IntegratedImuModel {
    /// The control input `u` is a 6D vector: [ax, ay, az, wx, wy, wz] from the IMU.
    fn get_control_dim(&self) -> usize {
        6
    }

    /// This is the key declarative method. It checks if the provided measurement
    /// data is an IMU reading and, if so, converts it into the control vector `u`.
    fn get_control_from_measurement(&self, data: &MeasurementData) -> Option<DVector<f64>> {
        match data {
            // Case 1: The data is from a 6-DOF IMU. We use it directly.
            MeasurementData::Imu6Dof(imu_vector) => {
                // Convert the static `Vector6` into a dynamic `DVector`.
                Some(DVector::from_row_slice(imu_vector.as_slice()))
            }
            // Case 2: The data is from a 9-DOF IMU. We only want the accel/gyro part.
            MeasurementData::Imu9Dof {
                base_data,
                magnetic_field: _,
            } => {
                // We explicitly ignore the `mag` field and only use the 6-DOF part.
                Some(DVector::from_row_slice(base_data.as_slice()))
            }
            // Case 3: The data is anything else (GPS, etc.). This model does not
            // use it as a control input, so we return None.
            _ => None,
        }
    }

    fn get_derivatives(&self, x: &State, u: &Control, _t: f64) -> State {
        let mut x_dot = DVector::zeros(16);

        // --- 1. Extract state variables and inputs  ---
        let velocity_world = x.fixed_rows::<3>(3);
        let orientation_quat = UnitQuaternion::new_normalize(
            Quaternion::new(x[9], x[6], x[7], x[8]), // w, x, y, z
        );
        let accel_bias = x.fixed_rows::<3>(10);
        // Corrected index for gyro_bias
        let gyro_bias = x.fixed_rows::<3>(13);

        let raw_accel_measurement = u.fixed_rows::<3>(0);
        let raw_gyro_measurement = u.fixed_rows::<3>(3);

        // --- 2. Correct the raw measurements  ---
        let corrected_accel_body = raw_accel_measurement - accel_bias;
        let corrected_gyro_body = raw_gyro_measurement - gyro_bias;

        // --- 3. Position and Velocity Derivatives  ---
        x_dot.fixed_rows_mut::<3>(0).copy_from(&velocity_world);
        let accel_world = orientation_quat * corrected_accel_body;
        let gravity_world = Vector3::new(0.0, 0.0, -self.gravity_magnitude);
        x_dot
            .fixed_rows_mut::<3>(3)
            .copy_from(&(accel_world + gravity_world));

        // println!("raw_accel_measurement: {:?}", raw_accel_measurement);
        // println!("corrected_accel_body: {:?}", corrected_accel_body);
        // println!("accel_world: {:?}", accel_world);
        // println!("gravity_world: {:?}", gravity_world);

        // --- 4. Orientation Derivative ---
        // Create the pure quaternion for the angular velocity.
        let omega_quat = nalgebra::Quaternion::from_imag(corrected_gyro_body);

        // Perform the quaternion multiplication first: `q * ω_q`.
        // The result is a `Quaternion<f64>`.
        let q_times_omega = *orientation_quat * omega_quat;

        // Now, scale the resulting quaternion by 0.5.
        let q_dot_quat = q_times_omega * 0.5;

        // Finally, extract the coordinates of the final result for the state vector derivative.
        x_dot.fixed_rows_mut::<4>(6).copy_from(&q_dot_quat.coords); // q_dot_quat.coords is [x, y, z, w]

        // --- 5. Bias Derivatives (this part is correct) ---
        // Derivatives remain zero, handled by process noise Q.

        x_dot
    }

    // Since this model is intended for an EKF, providing the Jacobian is crucial.
    // As it's highly complex, we will use the numerical finite differencing method.
    // The EKF itself can call this, or a generic helper can be created.

    fn calculate_jacobian(&self, x: &State, u: &Control, t: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        // --- Numerical Differentiation for the Dynamics Jacobians A and B ---

        let state_dim = 16;
        let control_dim = self.get_control_dim();
        let mut a_jac = DMatrix::zeros(state_dim, state_dim);
        let mut b_jac = DMatrix::zeros(state_dim, control_dim);

        let epsilon = 1e-7; // A small perturbation value

        // 1. Calculate the baseline state derivative with the current state and control.
        let x_dot_base = self.get_derivatives(x, u, t);

        // --- 2. Calculate Jacobian A (w.r.t. state x) ---
        for j in 0..state_dim {
            // Create a copy of the state vector to perturb.
            let mut x_perturbed = x.clone();
            x_perturbed[j] += epsilon;

            // Calculate the derivative with the perturbed state.
            let x_dot_perturbed = self.get_derivatives(&x_perturbed, u, t);

            // Approximate the partial derivative column: (f(x+h) - f(x)) / h
            let derivative_column = (x_dot_perturbed - &x_dot_base) / epsilon;

            a_jac.column_mut(j).copy_from(&derivative_column);
        }

        // --- 3. Calculate Jacobian B (w.r.t. control u) ---
        for j in 0..control_dim {
            // Create a copy of the control vector to perturb.
            let mut u_perturbed = u.clone();
            u_perturbed[j] += epsilon;

            // Calculate the derivative with the perturbed control.
            let x_dot_perturbed = self.get_derivatives(x, &u_perturbed, t);

            let derivative_column = (x_dot_perturbed - &x_dot_base) / epsilon;

            b_jac.column_mut(j).copy_from(&derivative_column);
        }

        (a_jac, b_jac)
    }
}

// NOTE: The `propagate` method from the trait uses the above `get_derivatives`
// with an integrator like RK4, so it does not need to be reimplemented.
