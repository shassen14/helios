// helios_sim/src/simulation/core/mod.rs

use crate::simulation::core::{
    components::GroundTruthState,
    topics::TopicBus,
    transforms::{bevy_global_transform_to_enu_iso, bevy_vector_to_enu_vector, TfFramePose, TfTree},
};
use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;
use std::sync::Arc;

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
        ground_truth.angular_velocity = bevy_vector_to_enu_vector(&Vec3::new(ang_vel.x, ang_vel.y, ang_vel.z));
        // TODO: Calculate angular acceleration here using the same pattern.

        // --- 2. Linear Velocity and Acceleration (The Correct Way) ---

        // Get the current velocity from the physics engine and convert to ENU.
        let current_linear_velocity_enu =
            bevy_vector_to_enu_vector(&Vec3::new(lin_vel.x, lin_vel.y, lin_vel.z));

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

/// Publishes each agent's GroundTruthState to the TopicBus under
/// `/{agent_name}/ground_truth`. Runs in SimulationSet::Validation,
/// after ground_truth_sync_system has written the latest physics values.
pub fn ground_truth_publish_system(
    query: Query<(&Name, &GroundTruthState)>,
    mut topic_bus: ResMut<TopicBus>,
) {
    for (name, gt) in &query {
        topic_bus.publish(
            &format!("/{}/ground_truth", name.as_str()),
            gt.clone(),
        );
    }
}

/// Publishes each tracked TF frame as a `TfFramePose` to `/tf/{frame_name}`.
///
/// Runs in `SimulationSet::Validation`, after the post-physics `tf_tree_builder_system`
/// rebuild in `StateSync`, so every snapshot reflects the latest physics state.
/// Foxglove discovers these topics lazily and plots world/local pose over time.
pub fn tf_publish_system(tf_tree: Res<TfTree>, mut topic_bus: ResMut<TopicBus>) {
    let sim_time = tf_tree.sim_time;

    for (entity, world_iso, local_iso, parent_entity) in tf_tree.iter_frames() {
        let Some(frame_name) = tf_tree.entity_to_name.get(&entity).cloned() else {
            continue;
        };
        let parent_frame: Arc<str> = parent_entity
            .and_then(|pe| tf_tree.entity_to_name.get(&pe).cloned())
            .unwrap_or_else(|| Arc::from("world"));

        let qw = world_iso.rotation.quaternion();
        let ql = local_iso.rotation.quaternion();

        let pose = TfFramePose {
            sim_time,
            frame_name: frame_name.clone(),
            parent_frame,
            pos_x: world_iso.translation.x,
            pos_y: world_iso.translation.y,
            pos_z: world_iso.translation.z,
            quat_x: qw.i,
            quat_y: qw.j,
            quat_z: qw.k,
            quat_w: qw.w,
            local_pos_x: local_iso.translation.x,
            local_pos_y: local_iso.translation.y,
            local_pos_z: local_iso.translation.z,
            local_quat_x: ql.i,
            local_quat_y: ql.j,
            local_quat_z: ql.k,
            local_quat_w: ql.w,
        };

        topic_bus.publish(&format!("/tf/{}", frame_name.as_ref()), pose);
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

pub use topics::TopicBusPlugin;
