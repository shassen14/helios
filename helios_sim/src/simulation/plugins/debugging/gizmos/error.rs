use bevy::prelude::*;

use crate::simulation::core::components::GroundTruthState;
use crate::simulation::core::transforms::enu_iso_to_bevy_transform;
use crate::simulation::plugins::autonomy::EstimatorComponent;
use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;
use helios_core::frames::FrameAwareState;

/// Draws a red line between the ground truth position and the estimated position.
pub fn draw_estimation_error_line(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    query: Query<(&GroundTruthState, &EstimatorComponent)>,
) {
    if !config.show_error_line {
        return;
    }
    for (gt, module) in &query {
        if let Some(estimated_pose) =
            module.0.get_state().and_then(|s: &FrameAwareState| s.get_pose_isometry())
        {
            let gt_bevy = enu_iso_to_bevy_transform(&gt.pose).translation;
            let est_bevy = enu_iso_to_bevy_transform(&estimated_pose).translation;
            gizmos.line(gt_bevy, est_bevy, Color::srgb(1.0, 0.0, 0.0));
        }
    }
}
