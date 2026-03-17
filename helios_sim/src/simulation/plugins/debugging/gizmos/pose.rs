use bevy::prelude::*;

use crate::simulation::core::components::GroundTruthState;
use crate::simulation::core::transforms::enu_body_iso_to_bevy_transform;
use crate::simulation::plugins::autonomy::EstimatorComponent;
use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;
use helios_core::frames::FrameAwareState;

/// Draws coordinate frame axes at the ground truth pose of each agent.
pub fn draw_ground_truth_gimbals(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    agent_query: Query<&GlobalTransform, With<GroundTruthState>>,
) {
    if !config.show_pose_gimbals {
        return;
    }
    for transform in &agent_query {
        gizmos.axes(*transform, 1.5);
    }
}

/// Draws coordinate frame axes at the estimated pose of each agent.
pub fn draw_estimated_pose_gimbals(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    module_query: Query<&EstimatorComponent>,
) {
    if !config.show_pose_gimbals {
        return;
    }
    for module in &module_query {
        if let Some(estimated_pose_enu) = module
            .0
            .get_state()
            .and_then(|s: &FrameAwareState| s.get_pose_isometry())
        {
            let bevy_transform = enu_body_iso_to_bevy_transform(&estimated_pose_enu);
            let global_transform = GlobalTransform::from(bevy_transform);
            let start = global_transform.translation();
            let length = 1.2;
            gizmos.line(
                start,
                start + global_transform.right() * length,
                Color::srgb(1.0, 0.2, 0.2),
            );
            gizmos.line(
                start,
                start + global_transform.up() * length,
                Color::srgb(0.2, 1.0, 0.2),
            );
            gizmos.line(
                start,
                start + global_transform.back() * length,
                Color::srgb(0.2, 0.2, 1.0),
            );
        }
    }
}
