// helios_core/src/models/impl_dynamics/ackermann.rs

use crate::{
    frames::{FrameId, StateVariable},
    models::dynamics::Dynamics,
    types::FrameHandle,
};
use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};

#[derive(Debug, Clone)]
pub struct AckermannKinematics {
    pub wheelbase: f64,
    /// The handle of the agent this model is for.
    pub agent_handle: FrameHandle,
}

impl EstiDynamics for AckermannKinematics {
    fn get_state_layout(&self) -> Vec<StateVariable> {
        let body = FrameId::Body(self.agent_handle);
        vec![
            StateVariable::Px(FrameId::World), // X position in World
            StateVariable::Py(FrameId::World), // Y position in World
            // Full 3D orientation is part of the state for robustness
            StateVariable::Qx(body.clone(), FrameId::World),
            StateVariable::Qy(body.clone(), FrameId::World),
            StateVariable::Qz(body.clone(), FrameId::World),
            StateVariable::Qw(body.clone(), FrameId::World),
            // Forward velocity is in the Body's own X-axis
            StateVariable::Vx(body.clone()),
        ]
    }

    fn get_control_dim(&self) -> usize {
        2 // [commanded_linear_acceleration, commanded_steering_angle]
    }

    fn get_derivatives(&self, x: &DVector<f64>, u: &DVector<f64>, _t: f64) -> DVector<f64> {
        let mut x_dot = DVector::zeros(7);

        // --- Extract from state vector `x` ---
        let q_x = x[2];
        let q_y = x[3];
        let q_z = x[4];
        let q_w = x[5];
        let rotation = UnitQuaternion::from_quaternion(Quaternion::new(q_w, q_x, q_y, q_z));
        let body_x_axis_in_world = rotation * Vector3::x(); // The direction the car is facing

        let current_velocity = x[6]; // This is Vx(Body)

        // --- Extract from control vector `u` ---
        let commanded_accel = u[0];
        let commanded_steering_angle = u[1];

        // --- Calculate State Derivatives `x_dot` ---
        // 1. Position Derivative: World Velocity = Body Velocity rotated into World Frame
        let world_velocity = body_x_axis_in_world * current_velocity;
        x_dot[0] = world_velocity.x; // Px_dot
        x_dot[1] = world_velocity.y; // Py_dot

        // 2. Orientation Derivative (Quaternion derivative)
        let yaw_rate = (current_velocity / self.wheelbase) * commanded_steering_angle.tan();
        let angular_velocity_body = Vector3::new(0.0, 0.0, yaw_rate); // Body Z is "up" for yaw

        let w_quat = Quaternion::new(
            0.0,
            angular_velocity_body.x,
            angular_velocity_body.y,
            angular_velocity_body.z,
        );
        let q_dot = (rotation.into_inner() * w_quat) * 0.5; // Note: different multiplication order for body rates

        x_dot[2] = q_dot.i;
        x_dot[3] = q_dot.j;
        x_dot[4] = q_dot.k;
        x_dot[5] = q_dot.w;

        // 3. Velocity Derivative
        x_dot[6] = commanded_accel; // d(Vx_body)/dt

        x_dot
    }

    fn calculate_jacobian(
        &self,
        x: &DVector<f64>,
        u: &DVector<f64>,
        _t: f64,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        // For complex models like this, especially with quaternion kinematics,
        // deriving the analytical Jacobian is very advanced and error-prone.
        // It is common practice to use numerical differentiation (finite differences)
        // to approximate the Jacobian as a starting point.
        // Returning a zero matrix is a placeholder.
        (DMatrix::zeros(7, 7), DMatrix::zeros(7, 2))
    }
}
