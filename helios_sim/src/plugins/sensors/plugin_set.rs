// helios_sim/src/simulation/plugins/sensors/plugin_set.rs
//
// HeliosSensorsPlugin: aggregates all sensor simulation plugins.
// Future: reads scenario agent configs and adds only the required sensor plugins.

use bevy::prelude::*;

use super::{
    gps::GpsPlugin, imu::ImuPlugin, magnetometer::MagnetometerPlugin,
    raycasting::RaycastingSensorPlugin,
};

/// Adds all sensor simulation plugins (IMU, GPS, magnetometer, raycasting LiDAR).
pub struct HeliosSensorsPlugin;

impl Plugin for HeliosSensorsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            ImuPlugin,
            GpsPlugin,
            MagnetometerPlugin,
            RaycastingSensorPlugin,
        ));
    }
}
