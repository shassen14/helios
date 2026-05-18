// helios_sim/src/plugins/autonomy/mod.rs
//
// EstimationPlugin — spawning + pipeline tick + odom frames.
// AutonomyPlugin   — thin wrapper re-exporting EstimationPlugin.

use crate::prelude::*;

pub mod components;
pub mod systems;

pub use components::{
    AutonomyPipelineComponent, OdomFrameOf, PathFollowingOutputComponent, SensorPublishChannel,
};

use systems::{run_pipeline_tick, spawn_autonomy_pipeline, spawn_odom_frames, update_odom_frames};

// =========================================================================
// == EstimationPlugin
// =========================================================================

pub struct EstimationPlugin;

impl Plugin for EstimationPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            (
                spawn_autonomy_pipeline.in_set(SceneBuildSet::ProcessBaseAutonomy),
                spawn_odom_frames.in_set(SceneBuildSet::ProcessDependentAutonomy),
            ),
        );
        app.add_systems(
            FixedUpdate,
            (run_pipeline_tick, update_odom_frames)
                .chain()
                .in_set(SimulationSet::Estimation)
                .run_if(in_state(AppState::Running)),
        );
    }
}

// =========================================================================
// == AutonomyPlugin
// =========================================================================

pub struct AutonomyPlugin;

impl Plugin for AutonomyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(EstimationPlugin);
    }
}
