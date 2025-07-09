// src/simulation/models/ackermann.rs

use crate::simulation::core::{
    abstractions::Dynamics,
    frames::{FrameId, StateVariable},
};
use bevy::ecs::entity::Entity;
use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};

#[derive(Debug, Clone)]
pub struct AckermannKinematics {
    pub wheelbase: f64,
    pub agent_entity: Entity,
}

impl Dynamics for AckermannKinematics {
    fn get_state_layout(&self) -> Vec<StateVariable> {
        // A simple 2D car model tracks position and heading.
        // let body = FrameId::Body(agent_entity);
        let body = FrameId::Body(self.agent_entity);

        vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py((FrameId::World)),
            StateVariable::Pz(FrameId::World), // Z position (assume constant for 2D car)
            StateVariable::Qx(body.clone(), FrameId::World),
            StateVariable::Qy(body.clone(), FrameId::World),
            StateVariable::Qz(body.clone(), FrameId::World),
            StateVariable::Qw(body.clone(), FrameId::World),
            StateVariable::Vx(body.clone()), // X velocity (forward in body frame)
        ]
    }

    fn get_control_dim(&self) -> usize {
        2 // Control inputs are now: [commanded_linear_acceleration, commanded_steering_angle]
    }

    fn get_derivatives(&self, x: &DVector<f64>, u: &DVector<f64>, _t: f64) -> DVector<f64> {
        let state_dim = self.get_state_dim();
        let mut x_dot = DVector::zeros(state_dim);

        // --- Extract state variables ---
        let _pos_x = x[0];
        let _pos_y = x[1];
        let _pos_z = x[2];
        let current_velocity_x = x[7]; // This is Vx(Body)

        let q_w = x[6];
        let q_x = x[3];
        let q_y = x[4];
        let q_z = x[5];
        let rotation_quat = UnitQuaternion::from_quaternion(Quaternion::new(q_w, q_x, q_y, q_z));
        let (_roll, _pitch, yaw) = rotation_quat.euler_angles(); // World yaw

        // --- Extract control inputs ---
        let commanded_accel = u[0];
        let commanded_steering_angle = u[1];

        // --- Calculate State Derivatives ---

        // 1. Position Derivatives (Velocity in World frame)
        // Convert body-frame Vx to world-frame Vx, Vy, Vz
        let world_velocity_x = current_velocity_x * yaw.cos();
        let world_velocity_y = current_velocity_x * yaw.sin();
        let world_velocity_z = 0.0; // Assuming 2D motion in Z

        x_dot[0] = world_velocity_x; // d(Px)/dt = Vx_world
        x_dot[1] = world_velocity_y; // d(Py)/dt = Vy_world
        x_dot[2] = world_velocity_z; // d(Pz)/dt = Vz_world

        // 2. Quaternion Derivatives (Angular Velocity)
        let yaw_rate = (current_velocity_x / self.wheelbase) * commanded_steering_angle.tan();
        let angular_velocity_body = Vector3::new(0.0, 0.0, yaw_rate); // Angular velocity in Body frame (roll, pitch, yaw rates)

        let w_quat = Quaternion::new(
            0.0,
            angular_velocity_body.x,
            angular_velocity_body.y,
            angular_velocity_body.z,
        );
        let q_current = rotation_quat.into_inner();
        let q_dot = (w_quat * q_current) * 0.5; // Quaternion derivative formula

        x_dot[3] = q_dot.i; // d(Qx)/dt
        x_dot[4] = q_dot.j; // d(Qy)/dt
        x_dot[5] = q_dot.k; // d(Qz)/dt
        x_dot[6] = q_dot.w; // d(Qw)/dt

        // 3. Velocity Derivative (Acceleration in Body frame)
        x_dot[7] = commanded_accel; // d(Vx_body)/dt = commanded_accel

        x_dot
    }

    /// Calculates the Jacobian matrices F = ∂f/∂x and B = ∂f/∂u.
    /// This is the core of the linearization for the EKF.
    fn calculate_jacobian(
        &self,
        x: &DVector<f64>,
        u: &DVector<f64>,
        _t: f64,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        // let state_dim = self.get_state_layout(Entity::from_bits(0)).len();
        let state_dim = self.get_state_dim();
        let control_dim = self.get_control_dim();

        let mut f_jacobian = DMatrix::zeros(state_dim, state_dim);
        let mut b_jacobian = DMatrix::zeros(state_dim, control_dim);

        // --- Extract state and control for readability ---
        let current_velocity_x = x[7];
        let q_w = x[6];
        let q_x = x[3];
        let q_y = x[4];
        let q_z = x[5];
        let rotation_quat = UnitQuaternion::from_quaternion(Quaternion::new(q_w, q_x, q_y, q_z));
        let (_roll, _pitch, yaw) = rotation_quat.euler_angles();

        let commanded_steering_angle = u[1];

        // --- Fill F (∂f/∂x) ---
        // d(Px_dot)/d(Vx_body)
        f_jacobian[(0, 7)] = yaw.cos();
        // d(Py_dot)/d(Vx_body)
        f_jacobian[(1, 7)] = yaw.sin();
        // d(Px_dot)/d(Q.yaw) (implicit, via yaw)
        // This is complex. For quaternions, it involves derivatives of quaternion multiplication.
        // Often, numerical differentiation (finite differences) is used for complex Jacobians
        // if analytical derivation is too hard.
        // For simplified 2D, you might linearize yaw_rate = (v/L)*tan(delta) directly wrt Yaw.

        // For quaternion derivatives, the Jacobian entries w.r.t. Q are complex
        // and typically involve the skew-symmetric matrix of the angular velocity.
        // d(Q_dot)/dQ is related to the angular velocity (which is state-dependent).
        // d(Q_dot)/d_angular_velocity_body is 0.5 * (quaternion multiplication matrix).

        // For a simplified Ackermann:
        // Jacobian of Q_dot w.r.t. Q: (0.5 * angular_velocity_body_skew_symmetric)
        // ... (this part is complex and often done numerically or with specialized libraries)

        // --- Fill B (∂f/∂u) ---
        // d(Vx_dot)/d(commanded_accel)
        b_jacobian[(7, 0)] = 1.0;

        // d(Q_dot)/d(commanded_steering_angle)
        let denom = commanded_steering_angle.cos().powi(2);
        let d_yaw_rate_d_steering = (current_velocity_x / self.wheelbase) / denom;
        // This affects Q_dot via the yaw_rate.
        // This part also involves quaternion math derivatives.
        // d(Q_dot)/d_yaw_rate * d_yaw_rate/d_steering_angle
        // ... (complex)

        (f_jacobian, b_jacobian)
    }
}
