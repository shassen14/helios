// src/controllers/simple_go_to.rs
use crate::simulation::traits::{Control, Controller, Dynamics, Goal, State};
use nalgebra::Vector2;
use std::f64::consts::PI;

/// A simple controller that steers towards a goal point and tries to maintain a target speed.
#[derive(Debug)]
pub struct SimpleGoToController {
    pub target_velocity: f64,
    pub lookahead_distance: f64, // How far ahead to look for steering angle calc
    pub kp_velocity: f64,        // Proportional gain for velocity control
    pub kp_steering: f64,        // Proportional gain for steering control
}

impl Controller for SimpleGoToController {
    fn calculate_control(
        &mut self,
        current_state: &State,           // Expected: [x, y, theta, v]
        goal: &Goal,                     // We only use goal.state's position [x, y]
        dynamics: Option<&dyn Dynamics>, // For getting control dims/limits potentially
        _t: f64,
    ) -> Control {
        let state_dim = dynamics.map_or(4, |d| d.get_state_dim()); // Default to 4 if no dynamics
        let control_dim = dynamics.map_or(2, |d| d.get_control_dim()); // Default to 2

        let mut u = Control::zeros(control_dim); // Initialize control vector [delta, a]

        if current_state.len() < state_dim || goal.state.len() < 2 {
            // Not enough state info, return zero control
            return u;
        }

        let current_pos = Vector2::new(current_state[0], current_state[1]);
        let current_theta = current_state[2];
        let current_velocity = current_state[3];

        let target_pos = Vector2::new(goal.state[0], goal.state[1]);

        // --- Acceleration Control ---
        let velocity_error = self.target_velocity - current_velocity;
        let desired_acceleration = (self.kp_velocity * velocity_error).clamp(-5.0, 2.0); // Basic P control with limits
        u[1] = desired_acceleration; // Set acceleration component

        // --- Steering Control (Pure Pursuit inspired) ---
        let vector_to_target = target_pos - current_pos;
        let distance_to_target = vector_to_target.norm();

        if distance_to_target < 0.5 {
            // Close enough to goal
            u[0] = 0.0; // Stop steering
            u[1] = -2.0 * current_velocity; // Apply braking near goal
        } else {
            // Angle from car's current heading to the target direction
            let angle_to_target = vector_to_target.y.atan2(vector_to_target.x);
            let mut angle_diff = angle_to_target - current_theta;

            // Normalize angle difference to [-PI, PI]
            while angle_diff > PI {
                angle_diff -= 2.0 * PI;
            }
            while angle_diff < -PI {
                angle_diff += 2.0 * PI;
            }

            // Calculate desired steering angle (simplified pure pursuit logic)
            // Aim to intercept the path towards the goal
            // let lookahead_point = target_pos; // Simplest: aim directly at goal
            // Or: find point along vector_to_target at lookahead_distance?

            // Simple P control for steering angle based on angle difference
            let desired_steering = (self.kp_steering * angle_diff).clamp(-PI / 4.0, PI / 4.0); // Limit steering command

            // TODO: Incorporate wheelbase (dynamics might be needed) for more accurate delta calculation from desired turning radius/angle diff
            // delta = atan(L * curvature) approx atan(L * 2*sin(angle_diff) / lookahead_distance)
            // For now, just use proportional control on angle_diff

            u[0] = desired_steering;
        }

        // Optional: Get max steering from dynamics if available?
        // if let Some(dyn_model) = dynamics { ... clamp using dyn_model params ... }

        u
    }
}
