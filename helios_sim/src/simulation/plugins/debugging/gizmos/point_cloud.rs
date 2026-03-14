use bevy::prelude::*;

use crate::simulation::plugins::debugging::components::{
    DebugSensorCache, DebugVisualizationConfig,
};

/// Draws cyan sphere markers at each cached world-space point cloud position.
pub fn draw_point_cloud(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    cache: Res<DebugSensorCache>,
) {
    if !config.show_point_cloud {
        return;
    }
    let color = Color::srgba(0.0, 1.0, 0.8, 0.9);
    for pts in cache.point_clouds.values() {
        for &pt in pts {
            gizmos.sphere(Isometry3d::from_translation(pt), 0.05, color);
        }
    }
}
