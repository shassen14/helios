use crate::{
    prelude::*,
    simulation::{core::topics::GroundTruthState, plugins::world_model::WorldModelComponent},
};
use bevy::prelude::*;
use nalgebra::{Isometry3, UnitQuaternion, Vector3};

// =========================================================================
// == Plugin Definition ==
// =========================================================================

pub struct StateErrorDebugPlugin;

impl Plugin for StateErrorDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate, // Run at a predictable rate
            log_state_estimation_error
                .in_set(SimulationSet::Validation) // A new set that runs late in the frame
                .run_if(in_state(AppState::Running)),
        );
    }
}

// =========================================================================
// == Error Logging System ==
// =========================================================================

/// Calculates and logs the error between the ground truth and estimated state.
fn log_state_estimation_error(
    // We need both the ground truth and the estimator component.
    // We query for the agent entity that has both.
    query: Query<(Entity, &GroundTruthState, &WorldModelComponent)>,
) {
    // This assumes a single agent for simplicity.
    for (entity, ground_truth, module) in &query {
        if let WorldModelComponent::Separate { estimator, .. } = module {
            let estimated_state = estimator.get_state();

            // --- 1. Get the Poses ---
            // Get the true pose from the GroundTruthState component.
            let true_pose: &Isometry3<f64> = &ground_truth.pose;

            // Get the estimated pose from the filter.
            let estimated_pose: Isometry3<f64> = match estimated_state.get_pose_isometry() {
                Some(pose) => pose,
                None => {
                    // The estimator's state is not yet valid.
                    warn!("Estimator state is missing pose information.");
                    return;
                }
            };

            // --- 2. Calculate Position Error ---
            // This is the simple Euclidean distance between the two position vectors.
            let position_error_vec =
                true_pose.translation.vector - estimated_pose.translation.vector;
            let position_error_magnitude = position_error_vec.norm();

            // --- 3. Calculate Attitude (Orientation) Error ---
            // This is more complex. The error between two quaternions/rotations
            // is itself a rotation. We want the angle of that rotation.
            let true_rot: &UnitQuaternion<f64> = &true_pose.rotation;
            let estimated_rot: &UnitQuaternion<f64> = &estimated_pose.rotation;

            // The error rotation that transforms the estimate to the truth is:
            // error_rot = true_rot * estimated_rot_inverse
            let error_rotation = true_rot * estimated_rot.inverse();

            // The angle of this error rotation is the total angular error.
            // .angle() returns the value in radians. We convert to degrees for readability.
            let attitude_error_degrees = error_rotation.angle().to_degrees();

            // --- 4. Print the Report ---
            info!(
                "Agent {:?} State Error | Pos Err: {:.3}m | Att Err: {:.3}°",
                entity, position_error_magnitude, attitude_error_degrees
            );
        }
    }
}
