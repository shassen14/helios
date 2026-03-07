// helios_sim/src/plugins/world_model/mod.rs

use crate::prelude::*;

// --- Sub-modules for organization ---
mod components;
mod systems;

// Re-export the public-facing components for easy use elsewhere.
pub use components::{OdomFrameOf, WorldModelComponent};

use systems::{
    spawn_odom_frames, spawn_world_model_modules, update_odom_frames,
    world_model_event_processor, world_model_mapping_system, world_model_output_publisher,
};

pub struct WorldModelPlugin;

impl Plugin for WorldModelPlugin {
    fn build(&self, app: &mut App) {
        app
            // --- SPAWNING ---
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    spawn_world_model_modules.in_set(SceneBuildSet::ProcessBaseAutonomy),
                    // ProcessDependentAutonomy runs after ProcessBaseAutonomy (WorldModelComponent
                    // exists) and before Finalize (where build_static_tf_maps reads all names).
                    // This guarantees the odom entity is present when the TF name maps are built.
                    spawn_odom_frames.in_set(SceneBuildSet::ProcessDependentAutonomy),
                ),
            )
            // --- RUNTIME ---
            .add_systems(
                FixedUpdate,
                (
                    world_model_event_processor,
                    world_model_mapping_system,
                    world_model_output_publisher,
                    // Sync the odom TF frame last, after EKF has processed all measurements.
                    update_odom_frames,
                )
                    .chain()
                    .in_set(SimulationSet::Estimation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}
