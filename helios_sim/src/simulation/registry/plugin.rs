// helios_sim/src/simulation/registry/plugin.rs
//
// Top-level plugin that initialises the AutonomyRegistry resource and
// loads all default algorithm registrations. Add this to HeliosSimulationPlugin
// BEFORE WorldModelPlugin so the registry is populated before any spawning runs.

use bevy::prelude::*;

use super::{
    adapters::DefaultAdaptersPlugin, controllers::DefaultControllersPlugin,
    dynamics::DefaultDynamicsPlugin, estimators::DefaultEstimatorsPlugin,
    mappers::DefaultMappersPlugin, slam::DefaultSlamPlugin, AutonomyRegistry,
};

pub struct AutonomyRegistryPlugin;

impl Plugin for AutonomyRegistryPlugin {
    fn build(&self, app: &mut App) {
        // Initialise the resource first; Default* plugins register into it.
        app.init_resource::<AutonomyRegistry>().add_plugins((
            DefaultDynamicsPlugin,
            DefaultEstimatorsPlugin,
            DefaultMappersPlugin,
            DefaultSlamPlugin,
            DefaultControllersPlugin,
            DefaultAdaptersPlugin,
        ));
    }
}
