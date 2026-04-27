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
use helios_core::frames::{FrameId, StateVariable};

use super::super::DebugVisualizationConfig;
use crate::simulation::plugins::autonomy::PathFollowingComponent;

const COLOR_PATH: Color = Color::srgb(1.0, 1.0, 0.0); // yellow
const COLOR_WAYPOINT: Color = Color::srgb(0.0, 1.0, 1.0); // cyan

/// Convert a 2D ENU world position (z=0) to Bevy world space for path gizmos.
#[inline]
fn enu_path_to_bevy(enu_x: f64, enu_y: f64) -> Vec3 {
    Vec3::new(enu_x as f32, 0.05, -(enu_y as f32))
}

pub fn draw_planned_path(
    config: Res<DebugVisualizationConfig>,
    query: Query<&PathFollowingComponent>,
    mut gizmos: Gizmos,
) {
    if !config.show_planned_path {
        return;
    }

    for pf in &query {
        // Draw the current path held by PathFollowingCore.
        if let Some(path) = pf.0.get_path() {
            if path.waypoints.len() >= 2 {
                let points: Vec<Vec3> = path
                    .waypoints
                    .iter()
                    .filter_map(|wp| {
                        let p = wp.state.get_vector3(&StateVariable::Px(FrameId::World))?;
                        Some(enu_path_to_bevy(p.x, p.y))
                    })
                    .collect();
                for window in points.windows(2) {
                    gizmos.line(window[0], window[1], COLOR_PATH);
                }
            }
        }

        // Draw the active lookahead waypoint as a sphere.
        if let Some(wp) = pf.0.get_lookahead_waypoint() {
            if let Some(p) = wp.state.get_vector3(&StateVariable::Px(FrameId::World)) {
                let pos = enu_path_to_bevy(p.x, p.y);
                gizmos.sphere(Isometry3d::from_translation(pos), 0.5, COLOR_WAYPOINT);
            }
        }
    }
}
