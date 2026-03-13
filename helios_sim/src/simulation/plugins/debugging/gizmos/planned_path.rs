// helios_sim/src/simulation/plugins/debugging/gizmos/planned_path.rs
//
// Draws the planned path as a polyline in ENU→Bevy-converted coordinates.
//
// Coordinate convention (matches transforms.rs enu_iso_to_bevy_transform):
//   ENU  (+X East,  +Y North, +Z Up)
//   Bevy (+X right, +Y up,   -Z forward)
// Mapping: bevy_x = enu_x,  bevy_y = enu_z,  bevy_z = -enu_y
//
// For path waypoints at ground level (enu_z ≈ 0): bevy_y ≈ 0.

use bevy::prelude::*;
use helios_runtime::stage::PipelineLevel;

use crate::simulation::plugins::autonomy::ControlPipelineComponent;
use super::super::DebugVisualizationConfig;

const COLOR_LOCAL_PATH: Color = Color::srgb(1.0, 1.0, 0.0);   // yellow
const COLOR_GLOBAL_PATH: Color = Color::srgb(1.0, 0.5, 0.0);  // orange
const COLOR_WAYPOINT: Color = Color::srgb(0.0, 1.0, 1.0);     // cyan

/// Convert a 2D ENU world position (z=0) to Bevy world space for path gizmos.
#[inline]
fn enu_path_to_bevy(enu_x: f64, enu_y: f64) -> Vec3 {
    Vec3::new(enu_x as f32, 0.05, -(enu_y as f32))
}

pub fn draw_planned_path(
    config: Res<DebugVisualizationConfig>,
    query: Query<&ControlPipelineComponent>,
    mut gizmos: Gizmos,
) {
    if !config.show_planned_path {
        return;
    }

    for control in &query {
        // Draw local path (yellow).
        draw_level_path(
            &control.0,
            &PipelineLevel::Local,
            COLOR_LOCAL_PATH,
            COLOR_WAYPOINT,
            &mut gizmos,
        );

        // Draw global path (orange) if different from local.
        draw_level_path(
            &control.0,
            &PipelineLevel::Global,
            COLOR_GLOBAL_PATH,
            COLOR_WAYPOINT,
            &mut gizmos,
        );
    }
}

fn draw_level_path(
    core: &helios_runtime::pipeline::ControlCore,
    level: &PipelineLevel,
    line_color: Color,
    wp_color: Color,
    gizmos: &mut Gizmos,
) {
    let Some(path) = core.get_cached_path(level) else {
        return;
    };
    if path.waypoints.len() < 2 {
        return;
    }

    // Draw polyline.
    let points: Vec<Vec3> = path
        .waypoints
        .iter()
        .map(|wp| enu_path_to_bevy(wp.state.vector[0], wp.state.vector[1]))
        .collect();

    for window in points.windows(2) {
        gizmos.line(window[0], window[1], line_color);
    }

    // Draw active look-ahead waypoint as a sphere.
    if let Some(wp) = core.get_active_lookahead_waypoint(level) {
        let pos = enu_path_to_bevy(wp.state.vector[0], wp.state.vector[1]);
        gizmos.sphere(
            Isometry3d::from_translation(pos),
            0.5,
            wp_color,
        );
    }
}
