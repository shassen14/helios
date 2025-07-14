// helios_sim/src/plugins/world_model/mod.rs

use crate::prelude::*;
use bevy::prelude::*;

// --- Sub-modules for organization ---
mod components;
mod systems;
mod types;

// Re-export the public-facing components for easy use elsewhere.
pub use components::WorldModelComponent;

use systems::{
    spawn_world_model_modules, world_model_input_gatherer, world_model_output_publisher,
    world_model_processor,
};
use types::FrameInputs;

pub struct WorldModelPlugin;

impl Plugin for WorldModelPlugin {
    fn build(&self, app: &mut App) {
        app
            // This resource is the temporary "bus" for inputs each frame.
            .init_resource::<FrameInputs>()
            // Add the one-time spawning system.
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                spawn_world_model_modules.in_set(SceneBuildSet::ProcessBaseAutonomy),
            )
            // Add the runtime pipeline systems.
            .add_systems(
                FixedUpdate,
                (
                    world_model_input_gatherer,
                    world_model_processor,
                    world_model_output_publisher,
                )
                    .chain()
                    // We run our entire pipeline in the Estimation set for now.
                    // This could be its own `Autonomy` set as we discussed.
                    .in_set(SimulationSet::Estimation),
            );
    }
}
