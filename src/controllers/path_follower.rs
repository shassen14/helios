// src/controllers/path_follower.rs
use crate::simulation::{
    components::{ControlInput, CurrentPath}, // Need path component
    traits::{Control, Controller, Dynamics, Goal, State},
};
use nalgebra::Vector2;
use std::f64::consts::PI;

#[derive(Debug)]
pub struct PathFollowerController {
    pub target_velocity: f64,
    pub lookahead_distance: f64,
    pub kp_velocity: f64,
    pub kp_steering: f64,
    pub waypoint_reached_threshold: f64,
}

// src/controllers/path_follower.rs
impl Controller for PathFollowerController {
    fn calculate_control(
        &mut self,
        current_state: &State,
        _goal: &Goal,
        dynamics: Option<&dyn Dynamics>,
        path_opt: Option<&mut CurrentPath>, // Receive Option<&mut CurrentPath>
        _t: f64,
    ) -> Control {
        let control_dim = dynamics.map_or(2, |d| d.get_control_dim());
        // TODO: Make wheelbase a parameter?
        let wheelbase = 2.7; // Default wheelbase
        let mut u = Control::zeros(control_dim); // [delta, a]

        // Check if path_opt contains a mutable reference to CurrentPath
        if let Some(path_component) = path_opt {
            // Now operate on path_component.0 (which is Vec<State>)
            if current_state.len() < 4 || path_component.0.is_empty() {
                // No path or insufficient state, return zero control
                u[1] = -1.0 * current_state[3]; // Apply some braking if no path
                return u;
            }

            // State variables in Simulation Frame
            let current_pos = Vector2::new(current_state[0], current_state[1]); // (sim_x, sim_z)
            let current_yaw = current_state[2];
            let current_velocity = current_state[3];

            // --- Waypoint Removal (logic is okay, operates on sim coords) ---
            while path_component.0.len() > 1 {
                let waypoint_state = &path_component.0[0]; // [sim_x, sim_z, ...]
                if waypoint_state.len() < 2 {
                    path_component.0.remove(0);
                    continue;
                }
                let waypoint_pos = Vector2::new(waypoint_state[0], waypoint_state[1]); // (sim_x, sim_z)
                if (waypoint_pos - current_pos).norm_squared()
                    < self.waypoint_reached_threshold.powi(2)
                {
                    path_component.0.remove(0);
                } else {
                    break;
                }
            }

            // Ensure path is not empty after removal
            if path_component.0.is_empty() {
                u[1] = -1.0 * current_velocity; // Brake if path becomes empty
                return u;
            }

            // --- Target Selection (Simple: next waypoint) ---
            let target_waypoint_state = &path_component.0[0];
            // Ensure target state has enough dimensions
            if target_waypoint_state.len() < 2 {
                u[1] = -1.0 * current_velocity; // Brake if target is invalid
                return u;
            }
            let target_pos = Vector2::new(target_waypoint_state[0], target_waypoint_state[1]); // (sim_x, sim_z)

            // --- Lookahead (Placeholder: use target_pos) ---
            let lookahead_point = target_pos; // Replace with proper lookahead calculation later

            // --- Acceleration Control (logic is okay) ---
            let velocity_error = self.target_velocity - current_velocity;
            let max_accel = 2.0;
            let max_decel = -5.0;
            u[1] = (self.kp_velocity * velocity_error).clamp(max_decel, max_accel);

            // --- Steering Control (Pure Pursuit Alpha Calculation - UPDATED ANGLE) ---
            let vector_to_lookahead = lookahead_point - current_pos; // Vector in (sim_x, sim_z) plane

            // Angle of the lookahead vector relative to +Z_sim (Forward) axis
            let angle_to_lookahead = vector_to_lookahead.x.atan2(vector_to_lookahead.y); // atan2(right, forward)

            // Angle difference relative to current vehicle heading (sim_yaw)
            let mut alpha = angle_to_lookahead - current_yaw;
            while alpha > PI {
                alpha -= 2.0 * PI;
            }
            while alpha < -PI {
                alpha += 2.0 * PI;
            }

            let distance_to_lookahead = vector_to_lookahead.norm();
            let lookahead_dist_actual = distance_to_lookahead.max(0.1);

            // Steering angle calculation (Pure Pursuit formula)
            let desired_steering_angle =
                (2.0 * wheelbase * alpha.sin() / lookahead_dist_actual).atan();

            let max_steer = PI / 4.0; // Get from dynamics if possible
            u[0] = desired_steering_angle.clamp(-max_steer, max_steer);
        } else {
            // No path component available for this entity
            if current_state.len() >= 4 {
                u[1] = -1.0 * current_state[3]; // Apply braking
            }
            return u;
        }

        u
    }
}
