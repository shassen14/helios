use bevy::prelude::*;

// --- Sub-modules for organization ---
mod components;
mod systems;

// Re-export the public component for use in other plugins (like sensor spawners).
pub use components::ShowDebugGizmos;

use crate::prelude::{AppState, SimulationSet};

/// A top-level plugin that brings in all debugging visualization tools.
pub struct DebuggingPlugin;

impl Plugin for DebuggingPlugin {
    fn build(&self, app: &mut App) {
        app
            // --- Add the runtime systems ---
            .add_systems(
                Update,
                (
                    // Systems for global hotkey toggles
                    systems::toggle_lidar_visuals,
                    systems::toggle_pose_gimbals,
                    // The actual drawing systems
                    systems::draw_lidar_rays,
                    systems::draw_ground_truth_gimbals,
                    systems::draw_estimated_pose_gimbals,
                    systems::draw_covariance_ellipsoids,
                )
                    .run_if(in_state(AppState::Running)),
            )
            // The state error logger can remain in FixedUpdate for consistent output.
            .add_systems(
                FixedUpdate,
                systems::log_state_estimation_error
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}
