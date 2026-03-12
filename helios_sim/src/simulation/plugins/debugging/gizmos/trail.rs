use std::collections::VecDeque;

use bevy::prelude::*;

use crate::simulation::core::components::GroundTruthState;
use crate::simulation::plugins::debugging::components::{DebugVisualizationConfig, PathTrail};

/// Appends the current world position to each agent's path trail ring buffer.
/// Runs in FixedUpdate / Validation so the sampling rate matches physics.
pub fn update_path_trail(
    mut commands: Commands,
    mut query: Query<(Entity, &GlobalTransform, Option<&mut PathTrail>), With<GroundTruthState>>,
) {
    for (entity, transform, trail_opt) in &mut query {
        let current_pos = transform.translation();
        if let Some(mut trail) = trail_opt {
            trail.positions.push_back(current_pos);
            while trail.positions.len() > trail.max_len {
                trail.positions.pop_front();
            }
        } else {
            let mut positions = VecDeque::new();
            positions.push_back(current_pos);
            commands.entity(entity).insert(PathTrail {
                positions,
                max_len: 300,
            });
        }
    }
}

/// Draws a green polyline tracing the recent path of each agent.
pub fn draw_path_trail(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    query: Query<&PathTrail>,
) {
    if !config.show_path_trail {
        return;
    }
    for trail in &query {
        let pts: Vec<Vec3> = trail.positions.iter().copied().collect();
        if pts.len() >= 2 {
            gizmos.linestrip(pts, Color::srgba(0.5, 1.0, 0.5, 0.7));
        }
    }
}
