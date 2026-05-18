// helios_sim/src/simulation/registry/plugin.rs
//
// Initialises the VehicleAdapterRegistry and RuntimeAutonomyRegistry resources.
// Add this to HeliosSimulationPlugin BEFORE WorldModelPlugin so registries are
// populated before any spawning runs.

use bevy::prelude::*;
use helios_runtime::registry::AutonomyRegistry;

use super::{adapters::DefaultAdaptersPlugin, VehicleAdapterRegistry};

/// Wraps the portable `helios_runtime::AutonomyRegistry` as a Bevy resource.
///
/// `build_pipeline()` in the spawn system reads this to assemble `AutonomyPipeline`.
#[derive(Resource)]
pub struct RuntimeAutonomyRegistry(pub AutonomyRegistry);

impl Default for RuntimeAutonomyRegistry {
    fn default() -> Self {
        Self(AutonomyRegistry::default())
    }
}

pub struct AutonomyRegistryPlugin;

impl Plugin for AutonomyRegistryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<VehicleAdapterRegistry>()
            .init_resource::<RuntimeAutonomyRegistry>()
            .add_plugins(DefaultAdaptersPlugin);
    }
}
