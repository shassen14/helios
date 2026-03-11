use bevy::prelude::*;

use crate::simulation::core::components::GroundTruthState;
use crate::simulation::core::transforms::enu_vector_to_bevy_vector;
use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;

/// Draws a yellow arrow showing the ground truth linear velocity.
pub fn draw_velocity_vector(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    query: Query<(&GroundTruthState, &GlobalTransform)>,
) {
    if !config.show_velocity {
        return;
    }
    for (gt, transform) in &query {
        let origin = transform.translation();
        let vel_bevy = enu_vector_to_bevy_vector(&gt.linear_velocity);
        if vel_bevy.length_squared() > 1e-6 {
            gizmos.arrow(origin, origin + vel_bevy, Color::srgb(1.0, 1.0, 0.0));
        }
    }
}
