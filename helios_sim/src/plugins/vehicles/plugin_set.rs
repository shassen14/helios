// helios_sim/src/simulation/plugins/vehicles/plugin_set.rs
//
// HeliosVehiclesPlugin: aggregates all vehicle plugins.
// Future: reads the scenario agent list and adds only the required vehicle plugins.

use bevy::prelude::*;

use super::ackermann::AckermannCarPlugin;

/// Adds all vehicle plugins.
pub struct HeliosVehiclesPlugin;

impl Plugin for HeliosVehiclesPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(AckermannCarPlugin);
    }
}
