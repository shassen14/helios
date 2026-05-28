// helios_sim/src/simulation/plugins/isolation/mock_estimator.rs
//
// MockGroundTruthEstimatorPlugin stub.
//
// The real oracle-passthrough estimator now lives in
// helios_runtime::pipeline::nodes::mock_oracle_estimator as a DAG node
// (`kind = "MockOracle"` in autonomy config). This plugin remains as an
// empty shell only because SimulationProfile's MappingOnly / PlanningOnly
// / ControlOnly variants still reference it. Slated for deletion in
// Phase 7 alongside SimulationProfile.

use bevy::prelude::*;

pub struct MockGroundTruthEstimatorPlugin;

impl Plugin for MockGroundTruthEstimatorPlugin {
    fn build(&self, _app: &mut App) {
        // Stub: ground-truth passthrough deferred to GroundTruthEstimatorNode (Step 9b).
    }
}
