// helios_sim/src/plugins/world_model/mod.rs

use crate::{
    prelude::*,
    simulation::{
        core::events::BevyMeasurementMessage,
        plugins::world_model::systems::world_model_mapping_system,
    },
};
use bevy::prelude::*;

// --- Sub-modules for organization ---
mod components;
mod systems;
mod types;

// Re-export the public-facing components for easy use elsewhere.
pub use components::WorldModelComponent;

use systems::{
    spawn_world_model_modules, world_model_event_processor, world_model_input_gatherer,
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
            // A. HIGH-FREQUENCY, EVENT-DRIVEN ESTIMATOR
            .add_systems(
                Update, // Runs as fast as possible, every render frame.
                world_model_event_processor.run_if(on_event::<BevyMeasurementMessage>), // Only runs if there's work to do.
            )
            // B. LOW-FREQUENCY, FIXED-RATE SYSTEMS
            .add_systems(
                FixedUpdate, // Runs at our configured rate (e.g., 64Hz).
                (
                    // These two systems are gated internally by the ModuleTimer.
                    world_model_mapping_system,
                    // A future slam_optimization_system would also go here.

                    // The output publisher can also run at this slower, predictable rate.
                    world_model_output_publisher,
                )
                    .chain()
                    .in_set(SimulationSet::Estimation),
            );
    }
}
