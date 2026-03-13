// helios_sim/src/simulation/plugins/world/plugin_set.rs
//
// HeliosWorldPlugin: aggregates all world environment plugins.
// Future: reads `[world] type` from scenario TOML to add domain-specific plugins
// (UnderwaterWorldPlugin, SpaceWorldPlugin, …) — zero changes to ProfiledSimulationPlugin.

use bevy::prelude::*;

use super::{AtmospherePlugin, TerrainPlugin, WorldObjectPlugin};

/// Adds all world environment plugins (terrain, atmosphere, world objects).
pub struct HeliosWorldPlugin;

impl Plugin for HeliosWorldPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((TerrainPlugin, AtmospherePlugin, WorldObjectPlugin));
    }
}
