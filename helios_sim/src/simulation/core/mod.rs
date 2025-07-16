// helios_sim/src/simulation/core/mod.rs

use crate::simulation::core::{
    topics::GroundTruthState,
    transforms::{bevy_global_transform_to_enu_iso, bevy_transform_to_enu_iso},
};
use avian3d::prelude::{
    AngularVelocity, ExternalForce, ExternalTorque, Gravity, LinearVelocity, Mass,
};
use bevy::prelude::*;
use nalgebra::Vector3;

fn ground_truth_sync_system(
    mut query: Query<(
        &GlobalTransform,
        &LinearVelocity,
        &AngularVelocity,
        &Mass,
        // Get the forces that our `drive` system applied.
        &ExternalForce,
        // We can also get torques for angular acceleration.
        &ExternalTorque,
        &mut GroundTruthState, // <-- Now mutable
    )>,
    gravity: Res<Gravity>,
) {
    for (transform, lin_vel, ang_vel, mass, ext_force, ext_torque, mut ground_truth) in &mut query {
        // --- 1. Pose and Velocity Conversion (This part is still correct) ---
        ground_truth.pose = bevy_global_transform_to_enu_iso(transform);
        let current_linear_velocity_enu =
            Vector3::new(lin_vel.x as f64, -lin_vel.z as f64, lin_vel.y as f64);
        let current_angular_velocity_enu =
            Vector3::new(ang_vel.x as f64, -ang_vel.z as f64, ang_vel.y as f64);

        ground_truth.linear_velocity = current_linear_velocity_enu;
        ground_truth.angular_velocity = current_angular_velocity_enu;
        // --- 2. Calculate Net Force and Coordinate Acceleration (The New, Better Way) ---

        // a. Start with the external forces applied by our controller (in Bevy world frame).
        let total_force_bevy = ext_force.force();

        // b. The physics engine also applies gravity. Add that to the net force.
        //    (Note: Avian might also apply other forces like friction, which are harder to get.
        //     This is a much better approximation than differentiating velocity).
        let gravity_force_bevy = gravity.0 * mass.0;
        let net_force_bevy = total_force_bevy + gravity_force_bevy;

        // c. Use Newton's Second Law: a = F_net / m
        let linear_acceleration_bevy = net_force_bevy / mass.0;

        // d. Convert the final coordinate acceleration to our ENU frame.
        let linear_acceleration_enu = Vector3::new(
            linear_acceleration_bevy.x as f64,
            -linear_acceleration_bevy.z as f64,
            linear_acceleration_bevy.y as f64,
        );

        // --- 3. Update the GroundTruthState component with the new, stable value ---
        ground_truth.linear_acceleration = linear_acceleration_enu;

        // --- (Optional but recommended) Angular Acceleration ---
        // You can do the same for angular acceleration using torques and inertia.
        // For now, let's keep it simple.
        // let net_torque_bevy = ext_torque.torque();
        // let angular_acceleration_bevy = ... (requires moment of inertia)
        // ground_truth.angular_acceleration = ...
    }
}

pub mod app_state;
pub mod components;
pub mod config;
pub mod events;
pub mod prng;
pub mod simulation_setup;
pub mod spawn_requests;
pub mod topics;
pub mod transforms;
