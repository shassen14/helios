// helios_sim/src/simulation/plugins/isolation/mock_estimator.rs
//
// MockGroundTruthEstimatorPlugin stub.
// Ground-truth passthrough will be restored via GroundTruthEstimatorNode in a later step.

use bevy::prelude::*;

pub struct MockGroundTruthEstimatorPlugin;

impl Plugin for MockGroundTruthEstimatorPlugin {
    fn build(&self, _app: &mut App) {
        // Stub: ground-truth passthrough deferred to GroundTruthEstimatorNode (Step 9b).
    }
}
