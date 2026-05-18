// helios_sim/src/simulation/plugins/path_following/mod.rs
//
// PathFollowingPlugin stub. Will be wired to AutonomyPipelineComponent in a later step.

use bevy::prelude::*;

use crate::prelude::AppState;
use crate::simulation::core::app_state::SimulationSet;

pub struct PathFollowingPlugin;

impl Plugin for PathFollowingPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate,
            path_following_stub
                .in_set(SimulationSet::PathFollowing)
                .run_if(in_state(AppState::Running)),
        );
    }
}

/// No-op stub. Path following will be wired to the pipeline in a later step.
fn path_following_stub() {}
