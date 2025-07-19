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
        &mut GroundTruthState, // We need mutable access to update it
    )>,
    time: Res<Time>, // We need the time delta
) {
    let dt = time.delta_secs_f64();
    if dt < 1e-6 {
        return;
    } // Avoid division by zero if the simulation is paused

    for (transform, lin_vel, ang_vel, mut ground_truth) in &mut query {
        // --- 1. Pose and Angular Velocity ---
        // This part is correct and remains.
        ground_truth.pose = bevy_global_transform_to_enu_iso(transform);
        ground_truth.angular_velocity =
            Vector3::new(ang_vel.x as f64, -ang_vel.z as f64, ang_vel.y as f64);
        // TODO: Calculate angular acceleration here using the same pattern.

        // --- 2. Linear Velocity and Acceleration (The Correct Way) ---

        // Get the current velocity from the physics engine and convert to ENU.
        let current_linear_velocity_enu =
            Vector3::new(lin_vel.x as f64, -lin_vel.z as f64, lin_vel.y as f64);

        // Calculate coordinate acceleration using the stored velocity from the PREVIOUS frame.
        let linear_acceleration_enu =
            (current_linear_velocity_enu - ground_truth.last_linear_velocity) / dt;

        // --- 3. Update ALL GroundTruthState fields ---
        ground_truth.linear_velocity = current_linear_velocity_enu;
        ground_truth.linear_acceleration = linear_acceleration_enu;

        // --- 4. CRITICAL STEP: Store the current velocity for the NEXT frame's calculation ---
        ground_truth.last_linear_velocity = current_linear_velocity_enu;
    }
}

pub mod app_state;
pub mod components;
pub mod events;
pub mod prng;
pub mod simulation_setup;
pub mod spawn_requests;
pub mod topics;
pub mod transforms;
