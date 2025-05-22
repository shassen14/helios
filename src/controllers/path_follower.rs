// src/controllers/path_follower.rs
use crate::{
    models::bicycle_kinematic::BicycleKinematicModel,
    simulation::{
        components::{ControlInput, CurrentPath}, // Need path component
        traits::{Control, Controller, Dynamics, Goal, State},
    },
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
        current_state: &State, // ENU: [x_e, y_n, yaw_enu, v_enu]
        _goal: &Goal,          // Final goal, less used by pure path follower
        dynamics: Option<&dyn Dynamics>,
        path_opt: Option<&mut CurrentPath>, // This is Option<&mut CurrentPath>
        _t: f64,
    ) -> Control {
        let control_dim = dynamics.map_or(2, |d| d.get_control_dim());
        // Extract wheelbase if dynamics model provides it (requires a method on Dynamics trait)
        // let wheelbase = dynamics.and_then(|d| d.get_wheelbase_param()).unwrap_or(2.7); // Example
        let wheelbase = 2.7; // Placeholder if not available from dynamics trait yet

        let mut u = Control::zeros(control_dim); // ENU: [delta_enu_steering, accel_enu]

        // --- LOGGING START ---
        println!(
            "PathFollowerController called. Entity ENU State: x={:.2}, y={:.2}, yaw={:.2}, v={:.2}",
            current_state.get(0).cloned().unwrap_or_default(),
            current_state.get(1).cloned().unwrap_or_default(),
            current_state
                .get(2)
                .cloned()
                .unwrap_or_default()
                .to_degrees(), // Log yaw in degrees
            current_state.get(3).cloned().unwrap_or_default()
        );

        if let Some(path_component_mut_ref) = path_opt {
            // path_component_mut_ref is &mut CurrentPath
            println!(
                "  Controller received path. Initial waypoints: {}. First few: {:?}",
                path_component_mut_ref.0.len(),
                path_component_mut_ref.0.iter().take(3).collect::<Vec<_>>() // Log first 3 waypoints
            );

            if current_state.len() < 4 || path_component_mut_ref.0.is_empty() {
                println!("  Controller: Insufficient state or empty path. Applying brake.");
                if current_state.len() >= 4 {
                    u[1] = -2.0 * current_state[3].max(0.0);
                } // Brake
                return u;
            }

            let current_pos_enu = Vector2::new(current_state[0], current_state[1]); // ENU East, North
            let current_yaw_enu = current_state[2];
            let current_velocity_enu = current_state[3];

            // --- Waypoint Removal Logic ---
            let initial_len_before_removal = path_component_mut_ref.0.len();
            while path_component_mut_ref.0.len() > 1 {
                // Keep at least the last segment/goal
                let next_waypoint_state = &path_component_mut_ref.0[0];
                if next_waypoint_state.len() < 2 {
                    // Should not happen if planner is correct
                    println!("  WARN: Invalid waypoint in path (len < 2), removing.");
                    path_component_mut_ref.0.remove(0);
                    continue;
                }
                let waypoint_pos_enu = Vector2::new(next_waypoint_state[0], next_waypoint_state[1]); // ENU East, North
                let distance_sq_to_waypoint = (waypoint_pos_enu - current_pos_enu).norm_squared();

                if distance_sq_to_waypoint < self.waypoint_reached_threshold.powi(2) {
                    println!(
                        "  Controller: Reached waypoint ENU({:.2},{:.2}). Dist: {:.2}m. Removing.",
                        waypoint_pos_enu.x,
                        waypoint_pos_enu.y,
                        distance_sq_to_waypoint.sqrt()
                    );
                    path_component_mut_ref.0.remove(0); // THIS MODIFIES THE COMPONENT
                } else {
                    // println!(
                    //     "  Controller: Next waypoint ENU({:.2},{:.2}) is {:.2}m away (threshold {:.2}m). Keeping.",
                    //     waypoint_pos_enu.x, waypoint_pos_enu.y, distance_sq_to_waypoint.sqrt(), self.waypoint_reached_threshold
                    // );
                    break; // Haven't reached the current target waypoint yet
                }
            }
            if path_component_mut_ref.0.len() < initial_len_before_removal {
                println!(
                    "  Controller: Path length after removal: {}",
                    path_component_mut_ref.0.len()
                );
            }

            if path_component_mut_ref.0.is_empty() {
                println!("  Controller: Path became empty after removal! Applying brake.");
                u[1] = -2.0 * current_velocity_enu.max(0.0); // Brake
                return u;
            }

            // --- Target Selection (Pure Pursuit Lookahead Point) ---
            // For now, let's just target the *current first waypoint* after removal.
            // Proper lookahead finds a point ON the path segment `lookahead_distance` away.
            let target_waypoint_state = &path_component_mut_ref.0[0];
            if target_waypoint_state.len() < 2 {
                /* Should be handled by loop above */
                return u;
            }
            let lookahead_point_enu =
                Vector2::new(target_waypoint_state[0], target_waypoint_state[1]);
            // println!("  Controller: Targeting ENU Lookahead Point: ({:.2}, {:.2})", lookahead_point_enu.x, lookahead_point_enu.y);

            // --- Acceleration Control (P-controller on velocity error) ---
            let velocity_error = self.target_velocity - current_velocity_enu;
            let max_accel = 2.0; // m/s^2
            let max_decel = -3.0; // m/s^2 (braking)
            let desired_acceleration =
                (self.kp_velocity * velocity_error).clamp(max_decel, max_accel);
            u[1] = desired_acceleration; // accel_enu

            // --- Steering Control (Pure Pursuit-like) ---
            // Vector from current ENU position to ENU lookahead point
            let vector_to_lookahead_enu = lookahead_point_enu - current_pos_enu;
            let distance_to_lookahead = vector_to_lookahead_enu.norm();

            if distance_to_lookahead < 0.1 {
                // Very close to the lookahead point
                println!("  Controller: Very close to lookahead point, minimal steering.");
                u[0] = 0.0; // Minimal steering
            } else {
                // Angle of the lookahead point relative to current ENU X-axis (East)
                let angle_to_lookahead_global_enu =
                    vector_to_lookahead_enu.y.atan2(vector_to_lookahead_enu.x);
                // Angle difference (alpha) between car's heading and direction to lookahead point
                let mut alpha_enu = angle_to_lookahead_global_enu - current_yaw_enu;
                // Normalize alpha to [-PI, PI]
                while alpha_enu > PI {
                    alpha_enu -= 2.0 * PI;
                }
                while alpha_enu < -PI {
                    alpha_enu += 2.0 * PI;
                }

                // Pure Pursuit steering angle formula: delta = atan(2 * L * sin(alpha) / ld)
                // ld is the lookahead distance (or actual distance if using a fixed lookahead point from path)
                // Here, distance_to_lookahead is the effective lookahead distance to our chosen point.
                let desired_steering_angle_enu =
                    (2.0 * wheelbase * alpha_enu.sin() / distance_to_lookahead.max(0.1)).atan(); // max(0.1) to avoid div by zero

                let max_steer = 35.0f64.to_radians(); // Get from dynamics if possible
                u[0] = self.kp_steering * desired_steering_angle_enu.clamp(-max_steer, max_steer);
            }
            println!(
                "  Controller Output (ENU): Steer={:.2}deg, Accel={:.2}m/s^2. Target Vel: {:.2}, Current Vel: {:.2}. DistToTarget: {:.2}",
                u[0].to_degrees(),
                u[1],
                self.target_velocity,
                current_velocity_enu,
                distance_to_lookahead
            );
        } else {
            println!("  Controller: Path component NOT found for this entity. Applying brake.");
            if current_state.len() >= 4 {
                u[1] = -2.0 * current_state[3].max(0.0);
            } // Brake
        }
        // --- LOGGING END ---
        u
    }
}
