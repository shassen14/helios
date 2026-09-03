// Initialises the RuntimeAutonomyRegistry resource. Add this to
// HeliosSimulationPlugin BEFORE WorldModelPlugin so the registry is populated
// before any spawning runs.
use helios_runtime::registry::AutonomyRegistry;

use bevy::prelude::*;

/// Wraps the portable `helios_runtime::AutonomyRegistry` as a Bevy resource.
///
/// `build_pipeline()` in the spawn system reads this to assemble `AutonomyPipeline`.
#[derive(Resource, Default)]
pub struct RuntimeAutonomyRegistry(pub AutonomyRegistry);

pub struct AutonomyRegistryPlugin;

impl Plugin for AutonomyRegistryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RuntimeAutonomyRegistry>();
    }
}
