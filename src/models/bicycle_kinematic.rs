// src/models/bicycle_kinematic.rs

use crate::simulation::components::{ControlVector, StateVector}; // Use StateVector alias if preferred
use crate::simulation::traits::{Control, Dynamics, State};
use nalgebra::{DMatrix, DVector};

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

        let sim_yaw = x[2];
        let v = x[3];
        let delta = u[0].clamp(-self.max_steer_angle, self.max_steer_angle);
        let a = u[1];

        let x_dot = v * sim_yaw.sin(); // Component along X_sim (Right)
        let z_dot = v * sim_yaw.cos(); // Component along Z_sim (Forward)
        let yaw_dot = if self.wheelbase.abs() < 1e-6 {
            0.0
        } else {
            (v * delta.tan()) / self.wheelbase
        };
        let v_dot = a;

        StateVector::from_vec(vec![x_dot, z_dot, yaw_dot, v_dot])
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

        let sim_yaw = x[2];
        let v = x[3];
        let delta = u[0].clamp(-self.max_steer_angle, self.max_steer_angle);
        let l = self.wheelbase;

        // A = df/dx = [ [df1/dx, df1/dz, df1/dyaw, df1/dv],
        //               [df2/dx, df2/dz, df2/dyaw, df2/dv],
        //               [df3/dx, df3/dz, df3/dyaw, df3/dv],
        //               [df4/dx, df4/dz, df4/dyaw, df4/dv] ]
        // f1 = v*sin(yaw) -> df1/dyaw = v*cos(yaw), df1/dv = sin(yaw)
        // f2 = v*cos(yaw) -> df2/dyaw = -v*sin(yaw), df2/dv = cos(yaw)
        // f3 = v*tan(delta)/L -> df3/dv = tan(delta)/L
        // f4 = a -> all zero partials w.r.t state

        let mut a_mat = DMatrix::<f64>::zeros(Self::STATE_DIM, Self::STATE_DIM);
        a_mat[(0, 2)] = v * sim_yaw.cos(); // df1/dyaw
        a_mat[(0, 3)] = sim_yaw.sin(); // df1/dv
        a_mat[(1, 2)] = -v * sim_yaw.sin(); // df2/dyaw
        a_mat[(1, 3)] = sim_yaw.cos(); // df2/dv
        a_mat[(2, 3)] = if l.abs() < 1e-6 { 0.0 } else { delta.tan() / l }; // df3/dv

        // B = df/du = [ [df1/ddelta, df1/da],
        //               [df2/ddelta, df2/da],
        //               [df3/ddelta, df3/da],
        //               [df4/ddelta, df4/da] ]
        // f1, f2 partials w.r.t u are 0
        // f3 = v*tan(delta)/L -> df3/ddelta = v / (L * cos(delta)^2)
        // f4 = a -> df4/da = 1

        let mut b_mat = DMatrix::<f64>::zeros(Self::STATE_DIM, Self::CONTROL_DIM);
        let cos_delta_sq = delta.cos().powi(2);
        b_mat[(2, 0)] = if l.abs() < 1e-6 || cos_delta_sq < 1e-6 {
            0.0
        } else {
            v / (l * cos_delta_sq)
        }; // df3/ddelta
        b_mat[(3, 1)] = 1.0; // df4/da

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

        // Desired angular velocity yaw_dot_d = v * tan(delta) / L
        // Solve for delta: delta = atan(yaw_dot_d * L / v)
        let yaw_dot_d = x_dot_desired[2];
        let delta_ff = if v.abs() < 1e-3 {
            if yaw_dot_d.abs() < 1e-3 {
                0.0
            } else {
                return None;
            }
        } else {
            (yaw_dot_d * self.wheelbase / v).atan()
        };
        let delta_ff_clamped = delta_ff.clamp(-self.max_steer_angle, self.max_steer_angle);

        // Optional Consistency Check: Do a_ff and delta_ff_clamped produce desired x_dot/z_dot?
        // x_dot_check = v * x[2].sin() // Check if this matches x_dot_desired[0] approximately
        // z_dot_check = v * x[2].cos() // Check if this matches x_dot_desired[1] approximately
        // If not, the desired derivatives might be kinematically inconsistent.

        Some(ControlVector::from_vec(vec![delta_ff_clamped, a_ff]))
    }
}
