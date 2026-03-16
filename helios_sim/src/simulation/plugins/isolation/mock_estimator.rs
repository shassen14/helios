// helios_sim/src/simulation/plugins/isolation/mock_estimator.rs
//
// MockGroundTruthEstimatorPlugin: replaces the real EKF/UKF estimator with a
// perfect ground-truth passthrough.  Used by MappingOnly, PlanningOnly, ControlOnly profiles.

use bevy::prelude::*;

use crate::prelude::AppState;
use crate::simulation::core::app_state::{SceneBuildSet, SimulationSet};
use crate::simulation::core::components::GroundTruthState;
use crate::simulation::plugins::autonomy::systems::{
    publish_autonomy_telemetry, publish_sensor_telemetry, route_sensor_messages, spawn_odom_frames,
    spawn_passthrough_pipeline, update_odom_frames,
};
use crate::simulation::plugins::autonomy::EstimatorComponent;

pub struct MockGroundTruthEstimatorPlugin;

impl Plugin for MockGroundTruthEstimatorPlugin {
    fn build(&self, app: &mut App) {
        app
            // --- SPAWNING: GT passthrough instead of real EKF/UKF ---
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    spawn_passthrough_pipeline.in_set(SceneBuildSet::ProcessBaseAutonomy),
                    spawn_odom_frames.in_set(SceneBuildSet::ProcessDependentAutonomy),
                ),
            )
            // --- RUNTIME ---
            // Chain: route sensor mailbox → inject GT state → update odom frames.
            .add_systems(
                FixedUpdate,
                (route_sensor_messages, gt_inject_system, update_odom_frames)
                    .chain()
                    .in_set(SimulationSet::Estimation)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(
                FixedUpdate,
                (publish_autonomy_telemetry, publish_sensor_telemetry)
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}

/// Syncs each agent's `EstimatorComponent` from `GroundTruthState` each tick.
fn gt_inject_system(
    time: Res<Time>,
    mut agent_query: Query<(&mut EstimatorComponent, &GroundTruthState)>,
) {
    let timestamp = time.elapsed_secs_f64();
    for (mut estimator, gt) in &mut agent_query {
        estimator
            .0
            .inject_ground_truth(&gt.pose, gt.linear_velocity, timestamp);
    }
}
