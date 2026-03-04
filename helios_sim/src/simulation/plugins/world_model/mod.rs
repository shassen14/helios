// helios_sim/src/plugins/world_model/mod.rs

use crate::prelude::*;

// --- Sub-modules for organization ---
mod components;
mod systems;

// Re-export the public-facing components for easy use elsewhere.
pub use components::WorldModelComponent;

use systems::{
    spawn_world_model_modules, world_model_event_processor, world_model_mapping_system,
    world_model_output_publisher,
};

pub struct WorldModelPlugin;

impl Plugin for WorldModelPlugin {
    fn build(&self, app: &mut App) {
        app
            // --- SPAWNING ---
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                // We keep a single spawner that sets up all world model components.
                spawn_world_model_modules.in_set(SceneBuildSet::ProcessBaseAutonomy),
            )
            // --- RUNTIME ---
            .add_systems(
                FixedUpdate,
                (
                    world_model_event_processor,
                    world_model_mapping_system,
                    world_model_output_publisher,
                )
                    .chain()
                    .in_set(SimulationSet::Estimation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}
