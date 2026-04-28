//! Core simulation infrastructure: TF tree and ground-truth publishing.
//!
//! Contains systems that run every tick regardless of profile: ground-truth physics sync
//! (`ground_truth_sync_system`) and the structural/incremental TF tree update systems.
//! All cross-boundary frame conversions delegate to [`transforms`].

use crate::simulation::core::{
    components::GroundTruthState,
    transforms::{EnuBodyPose, EnuVector},
};
use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;

pub fn ground_truth_sync_system(
    mut query: Query<(
        &GlobalTransform,
        &LinearVelocity,
        &AngularVelocity,
        &mut GroundTruthState,
    )>,
    time: Res<Time>,
) {
    let dt = time.delta_secs_f64();
    if dt < 1e-6 {
        return;
    }

    for (transform, lin_vel, ang_vel, mut ground_truth) in &mut query {
        ground_truth.pose = EnuBodyPose::from(transform).0;
        ground_truth.angular_velocity =
            EnuVector::from(Vec3::new(ang_vel.x, ang_vel.y, ang_vel.z)).0;

        let current_linear_velocity_enu =
            EnuVector::from(Vec3::new(lin_vel.x, lin_vel.y, lin_vel.z)).0;

        let linear_acceleration_enu =
            (current_linear_velocity_enu - ground_truth.last_linear_velocity) / dt;

        ground_truth.linear_velocity = current_linear_velocity_enu;
        ground_truth.linear_acceleration = linear_acceleration_enu;
        ground_truth.last_linear_velocity = current_linear_velocity_enu;
    }
}

pub mod app_state;
pub mod components;
pub mod events;
pub mod prng;
pub mod sim_runtime;
pub mod simulation_setup;
pub mod spawn_requests;
pub mod transforms;
