// helios_core/src/models/controls/ackermann.rs
//
// ControlDynamics implementation for an Ackermann-steering vehicle.
//
// State vector (7 elements):
//   [0] Px  — world-frame East position (m)
//   [1] Py  — world-frame North position (m)
//   [2] Qx  — body-to-world quaternion, x component
//   [3] Qy  — body-to-world quaternion, y component
//   [4] Qz  — body-to-world quaternion, z component
//   [5] Qw  — body-to-world quaternion, w component
//   [6] Vx  — body-frame forward velocity (m/s)
//
// Control vector (2 elements):
//   [0] commanded_accel         — longitudinal acceleration (m/s²)
//   [1] commanded_steering_angle — front-wheel steering angle (rad, positive = left)

use nalgebra::{DVector, Quaternion, UnitQuaternion, Vector3};

use super::ControlDynamics;

pub const STATE_DIM: usize = 7;
pub const CONTROL_DIM: usize = 2;

/// Ackermann bicycle kinematics as a `ControlDynamics` model.
///
/// Shares the same mathematical model as `AckermannKinematics` (EstimationDynamics),
/// but is a standalone struct so the two trait impls stay fully decoupled.
/// Both structs delegate to the same equations; neither copies the other's code.
#[derive(Debug, Clone)]
pub struct AckermannControlDynamics {
    /// Distance between front and rear axles (m).
    pub wheelbase: f64,
}

impl AckermannControlDynamics {
    pub fn new(wheelbase: f64) -> Self {
        Self { wheelbase }
    }
}

impl ControlDynamics for AckermannControlDynamics {
    fn get_state_dim(&self) -> usize {
        STATE_DIM
    }

    fn get_control_dim(&self) -> usize {
        CONTROL_DIM
    }

    /// Ackermann bicycle model kinematics.
    ///
    /// x_dot = f(x, u):
    ///   Px_dot = Vx * (rotation * e_x).x   (world-frame East velocity)
    ///   Py_dot = Vx * (rotation * e_x).y   (world-frame North velocity)
    ///   q_dot  = 0.5 * q ⊗ [0, ω_body]     (quaternion kinematics)
    ///   Vx_dot = commanded_accel
    fn get_derivatives(&self, x: &DVector<f64>, u: &DVector<f64>, _t: f64) -> DVector<f64> {
        let mut x_dot = DVector::zeros(STATE_DIM);

        // --- Extract state ---
        let q = UnitQuaternion::from_quaternion(Quaternion::new(
            x[5], // w
            x[2], // x
            x[3], // y
            x[4], // z
        ));
        let vx = x[6];

        // --- Extract control ---
        let accel = u[0];
        let steering = u[1];

        // --- Position derivatives ---
        let world_forward = q * Vector3::x();
        x_dot[0] = world_forward.x * vx;
        x_dot[1] = world_forward.y * vx;

        // --- Quaternion derivative: q_dot = 0.5 * q ⊗ ω_quat ---
        let yaw_rate = (vx / self.wheelbase) * steering.tan();
        let omega_body = Vector3::new(0.0, 0.0, yaw_rate); // FLU: yaw around +Z
        let omega_quat = Quaternion::new(0.0, omega_body.x, omega_body.y, omega_body.z);
        let q_dot = (q.into_inner() * omega_quat) * 0.5;
        x_dot[2] = q_dot.i;
        x_dot[3] = q_dot.j;
        x_dot[4] = q_dot.k;
        x_dot[5] = q_dot.w;

        // --- Velocity derivative ---
        x_dot[6] = accel;

        x_dot
    }

    // calculate_jacobian: uses the finite-difference default from ControlDynamics.
    // A complete analytical Jacobian requires quaternion chain-rule terms for the
    // position rows (∂Px_dot/∂q*, ∂Py_dot/∂q*) that couple through the rotation.
    // The FD default is accurate to ~1e-4 and sufficient for LQR linearization.
    // Override here if profiling shows it is a bottleneck.
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn straight_state(vx: f64) -> DVector<f64> {
        // Identity quaternion (facing East), position at origin
        let mut x = DVector::zeros(STATE_DIM);
        x[5] = 1.0; // qw = 1
        x[6] = vx;
        x
    }

    #[test]
    fn straight_driving_position_derivative() {
        let model = AckermannControlDynamics::new(2.7);
        let x = straight_state(10.0);
        let u = DVector::from_vec(vec![0.0, 0.0]);
        let xdot = model.get_derivatives(&x, &u, 0.0);

        // Facing East (FLU +X → ENU +X), so Px_dot = 10, Py_dot ≈ 0
        assert_abs_diff_eq!(xdot[0], 10.0, epsilon = 1e-10);
        assert_abs_diff_eq!(xdot[1], 0.0, epsilon = 1e-10);
        assert_abs_diff_eq!(xdot[6], 0.0, epsilon = 1e-10); // no acceleration
    }

    #[test]
    fn acceleration_updates_velocity_derivative() {
        let model = AckermannControlDynamics::new(2.7);
        let x = straight_state(0.0);
        let u = DVector::from_vec(vec![3.0, 0.0]); // 3 m/s² accel
        let xdot = model.get_derivatives(&x, &u, 0.0);
        assert_abs_diff_eq!(xdot[6], 3.0, epsilon = 1e-10);
    }

    #[test]
    fn jacobian_has_correct_shape() {
        let model = AckermannControlDynamics::new(2.7);
        let x = straight_state(5.0);
        let u = DVector::from_vec(vec![1.0, 0.3]);
        let (a, b) = model.calculate_jacobian(&x, &u, 0.0);
        assert_eq!(a.nrows(), STATE_DIM);
        assert_eq!(a.ncols(), STATE_DIM);
        assert_eq!(b.nrows(), STATE_DIM);
        assert_eq!(b.ncols(), CONTROL_DIM);
    }

    #[test]
    fn jacobian_b_accel_column_is_unit_velocity() {
        // ∂Vx_dot/∂accel should be 1.0; all other B[:,0] entries should be ~0.
        let model = AckermannControlDynamics::new(2.7);
        let x = straight_state(5.0);
        let u = DVector::from_vec(vec![0.0, 0.0]);
        let (_, b) = model.calculate_jacobian(&x, &u, 0.0);
        assert_abs_diff_eq!(b[(6, 0)], 1.0, epsilon = 1e-4);
        for i in 0..STATE_DIM {
            if i != 6 {
                assert_abs_diff_eq!(b[(i, 0)], 0.0, epsilon = 1e-4);
            }
        }
    }
}
