// src/simulation/models/ackermann.rs

use crate::simulation::core::dynamics::{Control, Dynamics, State, StateVariable};
use nalgebra::{DMatrix, DVector};

#[derive(Debug, Default)]
pub struct AckermannKinematics {
    pub wheelbase: f64,
}

impl Dynamics for AckermannKinematics {
    fn get_state_layout(&self) -> Vec<StateVariable> {
        // A simple 2D car model tracks position and heading.
        vec![StateVariable::Px, StateVariable::Py, StateVariable::Yaw]
    }

    fn get_control_dim(&self) -> usize {
        2 // Control inputs are [velocity, steering_angle]
    }

    fn get_derivatives(&self, x: &DVector<f64>, u: &DVector<f64>, _t: f64) -> DVector<f64> {
        let yaw = x[2];
        let velocity = u[0];
        let steering_angle = u[1];

        let x_dot = velocity * yaw.cos();
        let y_dot = velocity * yaw.sin();
        let yaw_dot = velocity * steering_angle.tan() / self.wheelbase;

        DVector::from_vec(vec![x_dot, y_dot, yaw_dot])
    }

    /// Calculates the Jacobian matrices F = ∂f/∂x and B = ∂f/∂u.
    /// This is the core of the linearization for the EKF.
    fn calculate_jacobian(&self, x: &State, u: &Control, _t: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        let state_dim = self.get_state_dim();
        let control_dim = self.get_control_dim();

        // Extract state and control variables for clarity
        let yaw = x[2];
        let velocity = u[0];
        let steering_angle = u[1];

        // --- Calculate F = ∂f/∂x ---
        let mut f_jacobian = DMatrix::zeros(state_dim, state_dim);
        // df/dYaw
        f_jacobian[(0, 2)] = -velocity * yaw.sin(); // ∂(v*cos(Yaw))/∂Yaw
        f_jacobian[(1, 2)] = velocity * yaw.cos(); // ∂(v*sin(Yaw))/∂Yaw
        // All other elements of F are 0, so we leave them.

        // --- Calculate B = ∂f/∂u ---
        let mut b_jacobian = DMatrix::zeros(state_dim, control_dim);
        // df/dVelocity
        b_jacobian[(0, 0)] = yaw.cos();
        b_jacobian[(1, 0)] = yaw.sin();
        b_jacobian[(2, 0)] = steering_angle.tan() / self.wheelbase;
        // df/dSteeringAngle
        let cos_steering = steering_angle.cos();
        b_jacobian[(2, 1)] = (velocity / self.wheelbase) / (cos_steering * cos_steering); // (v/L) * sec^2(δ)

        (f_jacobian, b_jacobian)
    }
}
