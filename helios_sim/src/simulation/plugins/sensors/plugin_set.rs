// helios_sim/src/simulation/plugins/sensors/plugin_set.rs
//
// HeliosSensorsPlugin: aggregates all sensor simulation plugins.
// Future: reads scenario agent configs and adds only the required sensor plugins.

use bevy::prelude::*;

use super::{
    gps::GpsPlugin, imu::ImuPlugin, magnetometer::MagnetometerPlugin,
    raycasting::RaycastingSensorPlugin,
};
use crate::prelude::AppState;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::plugins::debugging::{register_actions, DebugToggle};
use crate::simulation::plugins::debugging::key_action_registry::KeyActionRegistry;

/// Adds all sensor simulation plugins (IMU, GPS, magnetometer, raycasting LiDAR).
pub struct HeliosSensorsPlugin;

impl Plugin for HeliosSensorsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            ImuPlugin,
            GpsPlugin,
            MagnetometerPlugin,
            RaycastingSensorPlugin,
        ))
        .add_systems(OnEnter(AppState::Running), register_sensor_keys);
    }
}

fn register_sensor_keys(
    mut registry: ResMut<KeyActionRegistry>,
    scenario: Res<ScenarioConfig>,
) {
    let overrides = &scenario.debug.keybindings.0;
    register_actions(
        &mut registry,
        overrides,
        &[("toggle_point_cloud", KeyCode::F3, "F3 Point Cloud", DebugToggle::PointCloud)],
    );
}
