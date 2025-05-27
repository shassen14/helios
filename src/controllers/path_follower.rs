// src/controllers/path_follower.rs
use nalgebra::Vector2; // Assuming State is DVector, but we use Vector2 for 2D points
use std::f64::consts::PI as PI_F64;

use crate::simulation::{
    components::CurrentPath,
    traits::{Control, Controller, Dynamics, Goal, State},
};

#[derive(Debug)]
pub struct PathFollowerController {
    pub target_velocity: f64,
    pub lookahead_distance: f64, // Ld
    pub kp_velocity: f64,
    pub kp_steering: f64, // This might not be needed if using pure pursuit formula directly
    pub waypoint_reached_threshold: f64,
}

impl Controller for PathFollowerController {
    fn calculate_control(
        &mut self,
        current_state: &State,
        _goal: &Goal,
        dynamics_opt: Option<&dyn Dynamics>, // Changed from dynamics to dynamics_opt
        path_opt: Option<&mut CurrentPath>,
        _t: f64,
    ) -> Control {
        let control_dim = dynamics_opt.map_or(2, |d| d.get_control_dim());
        let mut u = Control::zeros(control_dim);

        // Extract parameters from dynamics model if possible
        // Use the DynamicsExtended trait for this
        // let extended_dynamics = dynamics_opt
        //     .and_then(|d| (&d as &dyn Any).downcast_ref::<Box<dyn DynamicsExtended>>())
        //     .map(|boxed_dyn| boxed_dyn.as_ref());
        //
        // let wheelbase = extended_dynamics
        //     .and_then(|ed| ed.get_wheelbase())
        //     .unwrap_or(2.7);
        // let max_steer_rad = extended_dynamics
        //     .and_then(|ed| ed.get_max_steer_angle_rad())
        //     .unwrap_or(35.0f64.to_radians());

        // println!("PathFollowerController called. ENU State: x={:.2}, y={:.2}, yaw={:.2}deg, v={:.2}",
        //          current_state[0], current_state[1], current_state[2].to_degrees(), current_state[3]);

        let wheelbase = 2.7;
        let max_steer_rad = 35.0f64.to_radians();

        if let Some(path_component) = path_opt {
            // path_component is &mut CurrentPath
            if current_state.len() < 4 || path_component.0.is_empty() {
                if current_state.len() >= 4 {
                    u[1] = -2.0 * current_state[3].max(0.0);
                }
                return u;
            }

            let current_pos_enu = Vector2::new(current_state[0], current_state[1]);
            let current_yaw_enu = current_state[2];
            let current_velocity_enu = current_state[3];

            // --- Waypoint Pruning ---
            while path_component.0.len() > 1 {
                let waypoint_enu = Vector2::new(path_component.0[0][0], path_component.0[0][1]);
                if (waypoint_enu - current_pos_enu).norm_squared()
                    < self.waypoint_reached_threshold.powi(2)
                {
                    path_component.0.remove(0);
                } else {
                    break;
                }
            }

            if path_component.0.is_empty() {
                u[1] = -2.0 * current_velocity_enu.max(0.0);
                return u;
            }

            // --- Calculate Lookahead Point ---
            let mut lookahead_point_enu: Vector2<f64> = Default::default();
            let mut found_lookahead = false;

            if path_component.0.len() == 1 {
                // Only one point left (the goal)
                lookahead_point_enu = Vector2::new(path_component.0[0][0], path_component.0[0][1]);
                found_lookahead = true;
            } else {
                for i in 0..(path_component.0.len() - 1) {
                    let p1_state = &path_component.0[i];
                    let p2_state = &path_component.0[i + 1];
                    if p1_state.len() < 2 || p2_state.len() < 2 {
                        continue;
                    } // Skip invalid waypoints

                    let p1 = Vector2::new(p1_state[0], p1_state[1]);
                    let p2 = Vector2::new(p2_state[0], p2_state[1]);

                    let d_segment = p2 - p1;
                    let d_car_p1 = p1 - current_pos_enu; // Vector from car to start of segment

                    let a = d_segment.norm_squared();
                    let b = 2.0 * d_car_p1.dot(&d_segment);
                    let c = d_car_p1.norm_squared() - self.lookahead_distance.powi(2);

                    let discriminant = b * b - 4.0 * a * c;

                    if discriminant >= 0.0 && a.abs() > 1e-6 {
                        // Check a to avoid division by zero
                        let sqrt_discriminant = discriminant.sqrt();
                        let t1 = (-b - sqrt_discriminant) / (2.0 * a);
                        let t2 = (-b + sqrt_discriminant) / (2.0 * a);

                        // Check if intersections are on the segment [p1, p2] (0 <= t <= 1)
                        // And prefer the one "further along" or simply the valid t2
                        if t2 >= 0.0 && t2 <= 1.0 {
                            lookahead_point_enu = p1 + t2 * d_segment;
                            found_lookahead = true;
                            break;
                        } else if t1 >= 0.0 && t1 <= 1.0 {
                            lookahead_point_enu = p1 + t1 * d_segment;
                            found_lookahead = true;
                            break;
                        }
                    }
                }
            }

            if !found_lookahead {
                // If no intersection found (e.g., end of path is closer than lookahead_distance),
                // target the last point of the path.
                if let Some(last_wp_state) = path_component.0.last() {
                    if last_wp_state.len() >= 2 {
                        lookahead_point_enu = Vector2::new(last_wp_state[0], last_wp_state[1]);
                    } else {
                        // Fallback if last waypoint is invalid
                        u[1] = -2.0 * current_velocity_enu.max(0.0);
                        return u;
                    }
                } else {
                    // Should not happen if path_component wasn't empty
                    u[1] = -2.0 * current_velocity_enu.max(0.0);
                    return u;
                }
            }
            // println!("  Controller: Targeting ENU Lookahead Point: ({:.2}, {:.2})", lookahead_point_enu.x, lookahead_point_enu.y);

            // --- Acceleration Control ---
            let velocity_error = self.target_velocity - current_velocity_enu;
            u[1] = (self.kp_velocity * velocity_error).clamp(-3.0, 2.0);

            // --- Steering Control ---
            let vector_to_lookahead_enu = lookahead_point_enu - current_pos_enu;
            let distance_to_lookahead = vector_to_lookahead_enu.norm();

            if distance_to_lookahead < 0.1 {
                u[0] = 0.0;
            } else {
                let angle_to_lookahead_global_enu =
                    vector_to_lookahead_enu.y.atan2(vector_to_lookahead_enu.x);
                let mut alpha_enu = angle_to_lookahead_global_enu - current_yaw_enu;
                while alpha_enu > PI_F64 {
                    alpha_enu -= 2.0 * PI_F64;
                }
                while alpha_enu < -PI_F64 {
                    alpha_enu += 2.0 * PI_F64;
                }

                // Original Pure Pursuit: delta = atan( (2 * L * sin(alpha)) / ld )
                // ld is self.lookahead_distance OR distance_to_lookahead (the actual dist to chosen point)
                // Using distance_to_lookahead is more robust if the found point is closer than Ld.
                let steering_numerator = 2.0 * wheelbase * alpha_enu.sin();
                let desired_steering_angle_enu =
                    (steering_numerator / distance_to_lookahead.max(0.1)).atan();

                // The kp_steering gain might not be standard for pure pursuit if alpha is directly used for curvature.
                // Here, we calculate desired_steering_angle first, then apply kp_steering.
                // Often, kp_steering is 1.0 if the formula gives the direct angle.
                // Or, if the formula calculates curvature k = 2*sin(alpha)/ld, then delta = atan(k*L).
                // Let's assume your kp_steering is for tuning the output of the formula.
                u[0] = (self.kp_steering * desired_steering_angle_enu)
                    .clamp(-max_steer_rad, max_steer_rad);
            }

            // println!("  Controller Output (ENU): Steer={:.2}deg, Accel={:.2}m/s^2. Alpha={:.2}deg. LookaheadDistActual={:.2}",
            //          u[0].to_degrees(), u[1], alpha_enu.to_degrees(), distance_to_lookahead);
        } else {
            if current_state.len() >= 4 {
                u[1] = -2.0 * current_state[3].max(0.0);
            }
        }
        u
    }
}
