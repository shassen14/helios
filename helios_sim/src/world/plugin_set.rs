// HeliosWorldPlugin: aggregates all world environment plugins.
// Future: reads `[world] type` from scenario TOML to add domain-specific plugins
// (UnderwaterWorldPlugin, SpaceWorldPlugin, …) — added here, transparent to HeliosSimulationPlugin.

use bevy::prelude::*;

use super::{AssetGatePlugin, AtmospherePlugin, TerrainPlugin, WorldLayoutPlugin};

/// Adds all world environment plugins (terrain, atmosphere, object layout)
/// and the gate that waits for their assets.
pub struct HeliosWorldPlugin;

impl Plugin for HeliosWorldPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            TerrainPlugin,
            AtmospherePlugin,
            WorldLayoutPlugin,
            AssetGatePlugin,
        ));
    }
}
