use crate::frames::layout::STANDARD_INS_STATE_DIM;
use crate::models::estimation::dynamics::EstimationDynamics;
use crate::prelude::MeasurementData;
use crate::types::{Control, FrameHandle, State};
use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};

/// A dynamics model that integrates raw IMU measurements (as control inputs)
/// to propagate a 156-state inertial navigation system (INS) state vector.
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
            MeasurementData::Imu9Dof { accel_gyro, mag: _ } => {
                // We explicitly ignore the `mag` field and only use the 6-DOF part.
                Some(DVector::from_row_slice(accel_gyro.as_slice()))
            }
            // Case 3: The data is anything else (GPS, etc.). This model does not
            // use it as a control input, so we return None.
            _ => None,
        }
    }

    fn get_derivatives(&self, x: &State, u: &Control, _t: f64) -> State {
        let mut x_dot = DVector::zeros(STANDARD_INS_STATE_DIM);

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

        let control_dim = self.get_control_dim();
        let mut a_jac = DMatrix::zeros(STANDARD_INS_STATE_DIM, STANDARD_INS_STATE_DIM);
        let mut b_jac = DMatrix::zeros(STANDARD_INS_STATE_DIM, control_dim);

        let epsilon = 1e-7; // A small perturbation value

        // 1. Calculate the baseline state derivative with the current state and control.
        let x_dot_base = self.get_derivatives(x, u, t);

        // --- 2. Calculate Jacobian A (w.r.t. state x) ---
        for j in 0..STANDARD_INS_STATE_DIM {
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

#[cfg(test)]
mod tests {
    //! Tests for [`IntegratedImuModel`].
    //!
    //! Properties validated:
    //! - Control routing: IMU variants accepted, all other sensor types rejected.
    //! - Derivatives: position rate = velocity, gravity correctly subtracts from
    //!   IMU acceleration, accel/gyro biases subtract from raw measurements.
    //! - Jacobian: correct 16×16 / 16×6 shape; velocity-to-position coupling ≈ 1.
    //! - Propagation: RK4 integration with gravity-compensating IMU keeps agent
    //!   stationary (position and velocity remain near zero).

    use super::*;
    use crate::messages::MeasurementData;
    use crate::types::FrameHandle;
    use crate::utils::integrators::RK4;
    use nalgebra::{DVector, Vector3, Vector6};

    const AGENT: FrameHandle = FrameHandle(1);
    const G: f64 = 9.81;

    fn make_model() -> IntegratedImuModel {
        IntegratedImuModel { agent_handle: AGENT, gravity_magnitude: G }
    }

    /// Builds a 16-element state vector with the identity quaternion at indices 6-9.
    ///
    /// Index layout mirrors [`standard_ins_state_layout`](crate::frames::layout::standard_ins_state_layout):
    /// 0-2 position, 3-5 velocity, 6-9 quaternion (Qx, Qy, Qz, Qw),
    /// 10-12 accel bias, 13-15 gyro bias.
    fn identity_state() -> DVector<f64> {
        let mut x = DVector::zeros(STANDARD_INS_STATE_DIM);
        x[9] = 1.0; // Qw = 1 → identity quaternion
        x
    }

    /// Returns a 6-element control vector with `az = g` to exactly counteract gravity.
    fn gravity_compensating_imu(g: f64) -> DVector<f64> {
        let mut u = DVector::zeros(6);
        u[2] = g;
        u
    }

    // ── Control routing ──────────────────────────────────────────────────────

    #[test]
    fn control_routing_accepts_imu6dof() {
        // IMU6Dof is this model's primary driving input.
        let model = make_model();
        let data = MeasurementData::Imu6Dof(Vector6::zeros());
        assert!(model.get_control_from_measurement(&data).is_some());
    }

    #[test]
    fn control_routing_accepts_imu9dof_extracts_six_dof_part() {
        // IMU9Dof should be accepted and only the accel/gyro 6-dof part returned.
        let model = make_model();
        let data = MeasurementData::Imu9Dof {
            accel_gyro: Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            mag: Vector3::zeros(),
        };
        let result = model.get_control_from_measurement(&data);
        assert!(result.is_some());
        let u = result.unwrap();
        assert_eq!(u.nrows(), 6, "control vector must have 6 elements");
        assert!((u[0] - 1.0).abs() < 1e-12, "accel_x must be preserved");
        assert!((u[5] - 6.0).abs() < 1e-12, "gyro_z must be preserved");
    }

    #[test]
    fn control_routing_ignores_gps() {
        // GPS data is not a control input for this dynamics model.
        let model = make_model();
        let data = MeasurementData::GpsPosition(Vector3::zeros());
        assert!(model.get_control_from_measurement(&data).is_none());
    }

    // ── Derivatives ──────────────────────────────────────────────────────────

    #[test]
    fn derivatives_position_rate_equals_velocity() {
        // With world velocity [2, 3, 1] and identity orientation, x_dot[0..3] = [2, 3, 1].
        let model = make_model();
        let mut x = identity_state();
        x[3] = 2.0; // vx
        x[4] = 3.0; // vy
        x[5] = 1.0; // vz
        let u = gravity_compensating_imu(G); // zero net acceleration
        let xdot = model.get_derivatives(&x, &u, 0.0);

        assert!((xdot[0] - 2.0).abs() < 1e-9, "px_dot should equal vx");
        assert!((xdot[1] - 3.0).abs() < 1e-9, "py_dot should equal vy");
        assert!((xdot[2] - 1.0).abs() < 1e-9, "pz_dot should equal vz");
    }

    #[test]
    fn derivatives_free_fall_gives_downward_acceleration() {
        // Zero IMU input with identity orientation: gravity alone drives velocity
        // downward at ~9.81 m/s². Horizontal components must stay zero.
        let model = make_model();
        let x = identity_state();
        let u = DVector::zeros(6);
        let xdot = model.get_derivatives(&x, &u, 0.0);

        assert!(
            xdot[5] < -9.0,
            "vz_dot should be strongly negative (gravity pull), got {}",
            xdot[5]
        );
        assert!(xdot[3].abs() < 1e-9, "vx_dot must be zero in free fall");
        assert!(xdot[4].abs() < 1e-9, "vy_dot must be zero in free fall");
    }

    #[test]
    fn derivatives_gravity_compensation_gives_zero_velocity_change() {
        // IMU reports [0, 0, 9.81] — exactly cancelling gravity in the world frame.
        // With identity orientation, velocity derivative must be ≈ zero.
        let model = make_model();
        let x = identity_state();
        let u = gravity_compensating_imu(G);
        let xdot = model.get_derivatives(&x, &u, 0.0);

        assert!(xdot[3].abs() < 1e-9, "vx_dot must be 0 with gravity compensation");
        assert!(xdot[4].abs() < 1e-9, "vy_dot must be 0 with gravity compensation");
        assert!(
            xdot[5].abs() < 1e-9,
            "vz_dot must be 0 with gravity compensation, got {}",
            xdot[5]
        );
    }

    #[test]
    fn derivatives_accel_bias_subtracts_from_raw_measurement() {
        // Accel bias z = 1.0 means corrected = raw - bias.
        // With raw_az = 9.81 and bias_az = 1.0: corrected = 8.81.
        // velocity_z derivative = 8.81 + (-9.81) = -1.0.
        let model = make_model();
        let mut x = identity_state();
        x[12] = 1.0; // accel_bias_z (index 12 in standard INS layout)
        let u = gravity_compensating_imu(G);
        let xdot = model.get_derivatives(&x, &u, 0.0);

        assert!(
            (xdot[5] - (-1.0)).abs() < 1e-6,
            "vz_dot with 1.0 z-bias should be -1.0, got {}",
            xdot[5]
        );
    }

    // ── Jacobian ─────────────────────────────────────────────────────────────

    #[test]
    fn jacobian_has_correct_dimensions() {
        // A is the state Jacobian (16×16), B is the control Jacobian (16×6).
        let model = make_model();
        let x = identity_state();
        let u = DVector::zeros(6);
        let (a_jac, b_jac) = model.calculate_jacobian(&x, &u, 0.0);

        assert_eq!(a_jac.nrows(), STANDARD_INS_STATE_DIM, "A rows");
        assert_eq!(a_jac.ncols(), STANDARD_INS_STATE_DIM, "A cols");
        assert_eq!(b_jac.nrows(), STANDARD_INS_STATE_DIM, "B rows");
        assert_eq!(b_jac.ncols(), 6, "B cols = control_dim (ax,ay,az,wx,wy,wz)");
    }

    #[test]
    fn jacobian_velocity_to_position_coupling_is_unity() {
        // A(i, i+3) for i in 0..3 encodes d(pos)/d(vel) = 1.
        // This is the clearest structural property of the INS dynamics.
        let model = make_model();
        let x = identity_state();
        let u = DVector::zeros(6);
        let (a_jac, _) = model.calculate_jacobian(&x, &u, 0.0);

        for (pos_idx, vel_idx) in [(0, 3), (1, 4), (2, 5)] {
            assert!(
                (a_jac[(pos_idx, vel_idx)] - 1.0).abs() < 1e-4,
                "A({pos_idx},{vel_idx}) = ∂pos_dot/∂vel ≈ 1.0, got {}",
                a_jac[(pos_idx, vel_idx)]
            );
        }
    }

    // ── Propagation ──────────────────────────────────────────────────────────

    #[test]
    fn propagate_with_gravity_compensation_keeps_agent_stationary() {
        // At rest with gravity-compensating IMU input, RK4 integration over 0.1 s
        // must leave position and velocity near zero.
        let model = make_model();
        let x = identity_state();
        let u = gravity_compensating_imu(G);
        let x_next = model.propagate(&x, &u, 0.0, 0.1, &RK4);

        for idx in 0..6 {
            assert!(
                x_next[idx].abs() < 1e-9,
                "state[{idx}] should remain 0 at rest, got {}",
                x_next[idx]
            );
        }
    }
}
