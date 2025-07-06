// In a new file: src/simulation/models/ackermann.rs

use crate::simulation::core::dynamics::{Dynamics, StateVariable};
use nalgebra::DVector;

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
}
