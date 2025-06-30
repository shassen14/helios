// src/models/ackermann_vehicle_model.rs
use bevy::prelude::{GlobalTransform, Quat, Vec3 as BevyVec3};
use nalgebra::Vector2;
use std::any::Any; // For as_any
use std::f64::consts::PI as PI_F64;
use std::fmt::Debug;

use crate::simulation::components::ControlInput;
use crate::simulation::traits::{Control, Dynamics, State}; // Import Dynamics and other necessary items
use crate::vehicles::vehicle_params::VehiclePhysicalParams;
use avian3d::prelude::{AngularVelocity, LinearVelocity};

#[derive(Debug, Clone, Default)] // Added Default
pub struct AckermannVehicleModel {
    // This model might be stateless if all params are passed in methods
    // Or it could store some configuration if needed.
}

impl AckermannVehicleModel {
    pub fn new() -> Self {
        Self {}
    }

    // This is the specific method we want to call after downcasting
    pub fn calculate_world_forces_and_torques_from_wheels(
        &self,
        vehicle_params: &VehiclePhysicalParams,
        current_bevy_transform: &GlobalTransform,
        current_bevy_linear_velocity: &LinearVelocity,
        current_bevy_angular_velocity: &AngularVelocity,
        control_input: &ControlInput, // [target_steer_angle_enu, target_throttle_factor (-1 to 1)]
    ) -> (BevyVec3, BevyVec3) {
        let car_tf = current_bevy_transform.compute_transform();
        let car_world_rotation: Quat = car_tf.rotation;
        let car_world_position: BevyVec3 = car_tf.translation;

        let target_front_wheel_steer_angle_enu =
            control_input.0.get(0).cloned().unwrap_or(0.0) as f32;
        let throttle_factor = control_input.0.get(1).cloned().unwrap_or(0.0) as f32;

        let car_forward_vector_world = car_world_rotation * -BevyVec3::Z;
        let car_right_vector_world = car_world_rotation * BevyVec3::X;

        let world_velocity_at_com = current_bevy_linear_velocity.0;
        let world_angular_velocity_vec = current_bevy_angular_velocity.0; // This is a Vec3

        let half_track_front = vehicle_params.track_width_front as f32 / 2.0;
        let half_track_rear = vehicle_params.track_width_rear as f32 / 2.0;

        let wheel_pos_fl_local = BevyVec3::new(
            half_track_front,
            0.0,
            -vehicle_params.front_axle_offset_from_center as f32,
        );
        let wheel_pos_fr_local = BevyVec3::new(
            -half_track_front,
            0.0,
            -vehicle_params.front_axle_offset_from_center as f32,
        );
        let wheel_pos_rl_local = BevyVec3::new(
            half_track_rear,
            0.0,
            vehicle_params.rear_axle_offset_from_center as f32,
        );
        let wheel_pos_rr_local = BevyVec3::new(
            -half_track_rear,
            0.0,
            vehicle_params.rear_axle_offset_from_center as f32,
        );

        let wheel_local_positions_on_chassis = [
            wheel_pos_fl_local,
            wheel_pos_fr_local,
            wheel_pos_rl_local,
            wheel_pos_rr_local,
        ];
        let mut total_world_force = BevyVec3::ZERO;
        let mut total_world_torque = BevyVec3::ZERO;

        for i in 0..4 {
            let wheel_local_pos_relative_to_com = wheel_local_positions_on_chassis[i];
            let wheel_world_contact_offset = car_world_rotation
                * (wheel_local_pos_relative_to_com - BevyVec3::Y * vehicle_params.wheel_radius);
            let wheel_world_contact_pos = car_world_position + wheel_world_contact_offset;

            let r_wheel_contact_from_com_world = wheel_world_contact_pos - car_world_position; // Vector from CoM to wheel contact
            let wheel_contact_world_velocity = world_velocity_at_com
                + world_angular_velocity_vec.cross(r_wheel_contact_from_com_world);

            let mut actual_wheel_steer_angle_bevy_y = 0.0; // Steering is around Bevy local Y of the wheel mount
            let cornering_stiffness;

            if i < 2 {
                // Front wheels
                // Convert ENU steer (around ENU Z) to Bevy steer (around Bevy Y for the wheel's pivot)
                // If ENU steer +ve = left turn (CCW from top) and Bevy Y rot +ve = left turn (CCW from top), sign is same.
                actual_wheel_steer_angle_bevy_y = target_front_wheel_steer_angle_enu.clamp(
                    -vehicle_params.max_steer_angle_rad as f32,
                    vehicle_params.max_steer_angle_rad as f32,
                );
                cornering_stiffness = vehicle_params.front_tire_cornering_stiffness;
            } else {
                // Rear wheels
                cornering_stiffness = vehicle_params.rear_tire_cornering_stiffness;
            }

            // Wheel's orientation in world space, considering car's orientation and wheel's steer angle
            // Steering rotation is around the car's local Y axis, applied at the wheel's pivot point.
            // For simplicity, assume wheel "kingpin" axis is aligned with car's local Y.
            let steer_rotation_quat = Quat::from_rotation_y(actual_wheel_steer_angle_bevy_y);
            let wheel_orientation_world = car_world_rotation * steer_rotation_quat; // Orientation of the steered wheel assembly

            let wheel_longitudinal_axis_world = wheel_orientation_world * -BevyVec3::Z; // Wheel's "forward"
            let wheel_lateral_axis_world = wheel_orientation_world * BevyVec3::X; // Wheel's "right"

            // Project contact point velocity onto wheel's axes
            let v_longitudinal_wheel =
                wheel_contact_world_velocity.dot(wheel_longitudinal_axis_world);
            let v_lateral_wheel = wheel_contact_world_velocity.dot(wheel_lateral_axis_world);

            let slip_angle = if v_longitudinal_wheel.abs() < 0.1 {
                // Low forward speed on wheel
                if v_lateral_wheel.abs() < 0.01 {
                    0.0
                }
                // Avoid NaN from atan(0/0)
                else {
                    (std::f32::consts::PI / 2.0) * v_lateral_wheel.signum()
                }
            } else {
                (v_lateral_wheel / v_longitudinal_wheel).atan() // atan(V_lat_wheel / V_long_wheel)
            };

            let lateral_force_magnitude = -cornering_stiffness * slip_angle;
            let lateral_force_world = wheel_lateral_axis_world * lateral_force_magnitude;

            let mut longitudinal_force_magnitude = 0.0;
            if i >= 2 {
                // RWD: Rear wheels are drive wheels
                longitudinal_force_magnitude +=
                    throttle_factor * (vehicle_params.max_grip_force_longitudinal / 2.0);
            }
            if throttle_factor < -0.01 && i >= 2 {
                // Braking on drive wheels example (can be all wheels)
                longitudinal_force_magnitude += throttle_factor.clamp(-1.0, 0.0)
                    * (vehicle_params.max_grip_force_longitudinal / 2.0);
            }

            let approx_normal_force_per_wheel = vehicle_params.mass * 9.81 / 4.0;
            let rolling_resistance_force_mag =
                vehicle_params.rolling_resistance_coefficient * approx_normal_force_per_wheel;

            if v_longitudinal_wheel.abs() > 0.01 {
                // Apply rolling resistance if moving
                longitudinal_force_magnitude -=
                    rolling_resistance_force_mag * v_longitudinal_wheel.signum();
            }

            let longitudinal_force_world =
                wheel_longitudinal_axis_world * longitudinal_force_magnitude;

            let total_force_on_wheel_world = lateral_force_world + longitudinal_force_world;
            total_world_force += total_force_on_wheel_world;
            total_world_torque += r_wheel_contact_from_com_world.cross(total_force_on_wheel_world);

            // Debug print for one wheel:
            // if i == 0 && (throttle_factor.abs() > 0.05 || target_front_wheel_steer_angle_enu.abs() > 0.05) {
            //     println!("FL Wheel: SteerCmd={:.1}deg, ActualSteer={:.1}deg, SlipAngle={:.1}deg, Vlong={:.1}, Vlat={:.1}, LatF={:.0}, LongF={:.0}",
            //         target_front_wheel_steer_angle_enu.to_degrees(),
            //         actual_wheel_steer_angle_bevy_y.to_degrees(),
            //         slip_angle.to_degrees(),
            //         v_longitudinal_wheel, v_lateral_wheel,
            //         lateral_force_magnitude, longitudinal_force_magnitude);
            // }
        }
        (total_world_force, total_world_torque)
    }
}

// Implement the generic Dynamics trait for AckermannVehicleModel
impl Dynamics for AckermannVehicleModel {
    fn get_state_dim(&self) -> usize {
        // This model doesn't directly manage a "state vector" in the same way
        // the kinematic model did for its ODE. Its "state" is the RigidBody's state.
        // For compatibility, we can return typical car state size.
        4 // e.g., [x, y, yaw, speed] conceptually, though not used by this model's core calc.
    }

    fn get_control_dim(&self) -> usize {
        2 // [target_steer_angle_enu, target_throttle_factor]
    }

    fn get_derivatives(&self, _x: &State, _u: &Control, _t: f64) -> State {
        // This method is NOT directly used by AckermannVehicleModel when integrated
        // with a physics engine like Avian, as Avian handles state integration.
        // We provide a dummy implementation or panic if called.
        // Alternatively, this could be used for a purely kinematic version of Ackermann.
        // For now, let's return zeros or panic to indicate it's not for direct integration.
        // eprintln!("WARN: get_derivatives called on AckermannVehicleModel; this model is intended for force/torque calculation with a physics engine.");
        State::zeros(self.get_state_dim())
    }

    // Implement as_any and as_any_mut
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

