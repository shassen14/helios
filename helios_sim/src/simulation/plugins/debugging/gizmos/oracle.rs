use bevy::prelude::*;

use helios_runtime::channels::oracle_pose_channel;
use nalgebra::Isometry3;

use crate::simulation::core::transforms::EnuBodyPose;
use crate::simulation::plugins::autonomy::AutonomyPipelineComponent;
use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;

/// Draws coordinate-frame axes at the **oracle** pose for each agent —
/// i.e. the pose pulled from the autonomy bus on `oracle/pose`, rather
/// than read directly from `GroundTruthState` / `GlobalTransform`.
///
/// This is the end-to-end verification path for Phase 2's oracle plumbing:
/// if these axes appear and track the agent, then the publisher in
/// `publish_oracle_channels_system` wrote the slot, the slot allocation
/// in `PortBus::new` from `BodyCapabilities.publishes` worked, and a
/// `bus.read(oracle_pose_channel())` here reads it back.
///
/// Distinguished visually from F1 (`draw_ground_truth_gimbals`) by a
/// different color set — magenta / yellow / cyan rather than the default
/// red/green/blue — so the two can be on simultaneously without
/// overlapping confusingly.
pub fn draw_oracle_pose_gimbals(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    pipeline_query: Query<&AutonomyPipelineComponent>,
) {
    if !config.show_oracle_pose {
        return;
    }

    for pipeline in &pipeline_query {
        let Some(stamped) = pipeline
            .0
            .bus()
            .read::<Isometry3<f64>>(oracle_pose_channel())
        else {
            // No oracle pose published yet (cold start) or no slot
            // allocated for this body. Nothing to draw.
            continue;
        };

        let bevy_transform = Transform::from(EnuBodyPose(stamped.value));
        let global_transform = GlobalTransform::from(bevy_transform);
        let start = global_transform.translation();
        let length = 1.0;

        // Magenta / yellow / cyan to distinguish from F1's red/green/blue.
        gizmos.line(
            start,
            start + global_transform.right() * length,
            Color::srgb(1.0, 0.2, 1.0),
        );
        gizmos.line(
            start,
            start + global_transform.up() * length,
            Color::srgb(1.0, 1.0, 0.2),
        );
        gizmos.line(
            start,
            start + global_transform.back() * length,
            Color::srgb(0.2, 1.0, 1.0),
        );
    }
}
