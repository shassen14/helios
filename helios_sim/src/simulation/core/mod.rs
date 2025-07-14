// helios_sim/src/simulation/core/mod.rs

use crate::simulation::core::{
    topics::GroundTruthState, transforms::bevy_global_transform_to_enu_iso,
};
use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;
use nalgebra::Vector3;

fn ground_truth_sync_system(
    mut query: Query<(
        &GlobalTransform,
        &LinearVelocity,
        &AngularVelocity,
        &mut GroundTruthState, // <-- Now mutable
    )>,
    time: Res<Time>, // We need the time delta
) {
    let dt = time.delta_secs_f64();
    if dt <= 1e-6 {
        return;
    } // Avoid division by zero if paused

    for (transform, lin_vel, ang_vel, mut ground_truth) in &mut query {
        // --- 1. Update Pose and Angular Velocity (as before) ---
        ground_truth.pose = bevy_global_transform_to_enu_iso(transform);
        ground_truth.angular_velocity =
            Vector3::new(ang_vel.x as f64, -ang_vel.z as f64, ang_vel.y as f64);

        // --- 2. Update Linear Velocity and Calculate Acceleration ---
        let current_linear_velocity_enu =
            Vector3::new(lin_vel.x as f64, -lin_vel.z as f64, lin_vel.y as f64);

        // Calculate coordinate acceleration in the ENU world frame.
        let linear_acceleration_enu =
            (current_linear_velocity_enu - ground_truth.last_linear_velocity) / dt;

        // --- 3. Update the GroundTruthState component ---
        ground_truth.linear_velocity = current_linear_velocity_enu;
        ground_truth.linear_acceleration = linear_acceleration_enu;

        // --- 4. IMPORTANT: Store the current velocity for the *next* frame's calculation ---
        ground_truth.last_linear_velocity = current_linear_velocity_enu;

        // --- (Optional but good) Angular Acceleration ---
        // You would add `last_angular_velocity` to GroundTruthState and do the same calculation here.
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
