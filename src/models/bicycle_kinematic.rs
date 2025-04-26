// src/models/bicycle_kinematic.rs

use crate::simulation::components::{ControlVector, StateVector}; // Use StateVector alias if preferred
use crate::simulation::traits::{Control, Dynamics, State};
use nalgebra::{DMatrix, DVector};
use std::f64::consts::PI; // Use f64 PI

/// Kinematic Bicycle Model for car-like vehicles.
/// Assumes rear-wheel drive, steering on front wheels.
/// State vector: [x, y, theta, v] (position_x, position_y, heading_angle, longitudinal_velocity)
/// Control input: [delta, a] (steering_angle, longitudinal_acceleration)
#[derive(Debug, Clone)] // Add Debug, Clone might be useful
pub struct BicycleKinematicModel {
    /// Distance between front and rear axles (meters).
    pub wheelbase: f64,
    /// Maximum allowable steering angle (radians).
    pub max_steer_angle: f64,
}

impl BicycleKinematicModel {
    pub fn new(wheelbase: f64, max_steer_angle_deg: f64) -> Self {
        assert!(wheelbase > 0.0, "Wheelbase must be positive.");
        Self {
            wheelbase,
            max_steer_angle: max_steer_angle_deg.to_radians(),
        }
    }

    const STATE_DIM: usize = 4;
    const CONTROL_DIM: usize = 2;
}

impl Dynamics for BicycleKinematicModel {
    fn get_state_dim(&self) -> usize {
        Self::STATE_DIM
    }

    fn get_control_dim(&self) -> usize {
        Self::CONTROL_DIM
    }

    /// Calculates x_dot = f(x, u, t)
    /// x = [x, y, theta, v]
    /// u = [delta, a]
    /// x_dot = [v*cos(theta), v*sin(theta), v*tan(delta)/L, a]
    fn get_derivatives(&self, x: &State, u: &Control, _t: f64) -> State {
        // --- Input Validation & Clamping ---
        // Ensure state and control vectors have correct dimensions
        if x.nrows() != Self::STATE_DIM || u.nrows() != Self::CONTROL_DIM {
            // Log error and return zero derivatives
            eprintln!("Error: BicycleKinematicModel received incorrect state/control dimensions.");
            return StateVector::zeros(Self::STATE_DIM);
        }

        let theta = x[2]; // Heading angle
        let v = x[3]; // Longitudinal velocity

        // Clamp steering angle
        let delta = u[0].clamp(-self.max_steer_angle, self.max_steer_angle);
        let a = u[1]; // Acceleration

        // --- Calculate Derivatives ---
        let x_dot = v * theta.cos();
        let y_dot = v * theta.sin();
        // Angular velocity: v * tan(delta) / L
        // Handle potential division by zero if wheelbase is zero (already asserted in new)
        // Handle tan(delta) potentially becoming very large near +/- PI/2 (max_steer_angle should prevent this)
        let theta_dot = if self.wheelbase.abs() < 1e-6 {
            0.0
        } else {
            (v * delta.tan()) / self.wheelbase
        };
        let v_dot = a; // Acceleration directly controls velocity change rate

        StateVector::from_vec(vec![x_dot, y_dot, theta_dot, v_dot])
    }

    /// Calculates Jacobians A = df/dx, B = df/du (Optional but useful for EKF/LQR)
    fn calculate_jacobian(&self, x: &State, u: &Control, _t: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        // --- Input Validation & Clamping --- (same as get_derivatives)
        if x.nrows() != Self::STATE_DIM || u.nrows() != Self::CONTROL_DIM {
            eprintln!(
                "Error: BicycleKinematicModel Jacobian received incorrect state/control dimensions."
            );
            // Return zero matrices of appropriate size
            return (
                DMatrix::zeros(Self::STATE_DIM, Self::STATE_DIM),
                DMatrix::zeros(Self::STATE_DIM, Self::CONTROL_DIM),
            );
        }

        let theta = x[2];
        let v = x[3];
        let delta = u[0].clamp(-self.max_steer_angle, self.max_steer_angle);
        let l = self.wheelbase;

        // A = df/dx = [ [df1/dx, df1/dy, df1/dtheta, df1/dv],
        //               [df2/dx, df2/dy, df2/dtheta, df2/dv],
        //               [df3/dx, df3/dy, df3/dtheta, df3/dv],
        //               [df4/dx, df4/dy, df4/dtheta, df4/dv] ]
        // f1 = v*cos(theta) -> df1/dtheta = -v*sin(theta), df1/dv = cos(theta)
        // f2 = v*sin(theta) -> df2/dtheta =  v*cos(theta), df2/dv = sin(theta)
        // f3 = v*tan(delta)/L -> df3/dv = tan(delta)/L
        // f4 = a -> df4/dx = df4/dy = df4/dtheta = df4/dv = 0

        let mut a_mat = DMatrix::<f64>::zeros(Self::STATE_DIM, Self::STATE_DIM);
        a_mat[(0, 2)] = -v * theta.sin();
        a_mat[(0, 3)] = theta.cos();
        a_mat[(1, 2)] = v * theta.cos();
        a_mat[(1, 3)] = theta.sin();
        a_mat[(2, 3)] = if l.abs() < 1e-6 { 0.0 } else { delta.tan() / l };
        // Row 3 (index 3) is all zeros as df4/dx_i = 0

        // B = df/du = [ [df1/ddelta, df1/da],
        //               [df2/ddelta, df2/da],
        //               [df3/ddelta, df3/da],
        //               [df4/ddelta, df4/da] ]
        // f1 = v*cos(theta) -> df1/ddelta = 0, df1/da = 0
        // f2 = v*sin(theta) -> df2/ddelta = 0, df2/da = 0
        // f3 = v*tan(delta)/L -> df3/ddelta = v * sec(delta)^2 / L = v / (L * cos(delta)^2), df3/da = 0
        // f4 = a -> df4/ddelta = 0, df4/da = 1

        let mut b_mat = DMatrix::<f64>::zeros(Self::STATE_DIM, Self::CONTROL_DIM);
        let cos_delta_sq = delta.cos().powi(2);
        b_mat[(2, 0)] = if l.abs() < 1e-6 || cos_delta_sq < 1e-6 {
            0.0
        } else {
            v / (l * cos_delta_sq)
        };
        b_mat[(3, 1)] = 1.0;

        (a_mat, b_mat)
    }

    /// Calculates feedforward control `u_ff = [delta, a]` to achieve `x_dot_desired`.
    /// Solves x_dot_desired = f(x, u_ff, t) for u_ff.
    /// Input: x_dot_desired = [x_dot_d, y_dot_d, theta_dot_d, v_dot_d]
    fn calculate_feedforward_input(
        &self,
        x: &State,
        x_dot_desired: &State,
        _t: f64,
    ) -> Option<Control> {
        if x.nrows() != Self::STATE_DIM || x_dot_desired.nrows() != Self::STATE_DIM {
            eprintln!(
                "Error: BicycleKinematicModel feedforward received incorrect state dimensions."
            );
            return None;
        }

        let v = x[3]; // Current velocity

        // Desired acceleration 'a' is directly given by desired v_dot
        let a_ff = x_dot_desired[3];

        // Desired angular velocity theta_dot_d = v * tan(delta) / L
        // Solve for delta: tan(delta) = theta_dot_d * L / v
        // delta = atan(theta_dot_d * L / v)
        let theta_dot_d = x_dot_desired[2];
        let delta_ff = if v.abs() < 1e-3 {
            // If velocity is near zero, steering has no effect on theta_dot.
            // Cannot determine feedforward delta. Return None or 0? Let's return None.
            // Or, if theta_dot_d is also near zero, any delta is fine, return 0.
            if theta_dot_d.abs() < 1e-3 {
                0.0
            } else {
                return None;
            }
        } else {
            (theta_dot_d * self.wheelbase / v).atan()
        };

        // Clamp the calculated feedforward steering angle
        let delta_ff_clamped = delta_ff.clamp(-self.max_steer_angle, self.max_steer_angle);

        // Consistency Check (Optional but good):
        // Do the calculated ff controls produce the desired x_dot and y_dot?
        // x_dot_check = v * x[2].cos()
        // y_dot_check = v * x[2].sin()
        // If abs(x_dot_check - x_dot_desired[0]) > tolerance or abs(y_dot_check - x_dot_desired[1]) > tolerance -> return None?
        // This indicates the desired x_dot/y_dot are inconsistent with the achievable rates given v and theta_dot_d.

        Some(ControlVector::from_vec(vec![delta_ff_clamped, a_ff]))
    }
}
