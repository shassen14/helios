// helios_sim/src/simulation/plugins/path_following/mod.rs
//
// PathFollowingPlugin: advances the path follower cursor each tick and writes
// one TrajectoryPoint to PathFollowingOutputComponent for the control system.

use bevy::prelude::*;
use helios_core::frames::FrameAwareState;
use helios_core::types::FrameHandle;

use crate::prelude::AppState;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::components::{ControllerStateSource, GroundTruthState};
use crate::simulation::plugins::autonomy::{
    EstimatorComponent, PathFollowingComponent, PathFollowingOutputComponent,
};

pub struct PathFollowingPlugin;

impl Plugin for PathFollowingPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate,
            path_following_system
                .in_set(SimulationSet::PathFollowing)
                .run_if(in_state(AppState::Running)),
        );
    }
}

fn path_following_system(
    time: Res<Time>,
    mut query: Query<(
        Entity,
        &EstimatorComponent,
        Option<&mut PathFollowingComponent>,
        &mut PathFollowingOutputComponent,
        &ControllerStateSource,
        Option<&GroundTruthState>,
    )>,
) {
    let dt = time.delta_secs_f64();
    let ts = time.elapsed_secs_f64();

    for (entity, estimator, pf_opt, mut output, state_source, gt_opt) in &mut query {
        let gt_built: Option<FrameAwareState> = match state_source {
            ControllerStateSource::GroundTruth => {
                let handle = FrameHandle::from_entity(entity);
                gt_opt.map(|gt| gt.to_frame_aware_state(handle, ts))
            }
            ControllerStateSource::Estimated => None,
        };
        let state: Option<&FrameAwareState> = match state_source {
            ControllerStateSource::GroundTruth => gt_built.as_ref(),
            ControllerStateSource::Estimated => estimator.0.get_state(),
        };

        let Some(state) = state else {
            output.0 = None;
            continue;
        };

        output.0 = match pf_opt {
            Some(mut pf) => pf.0.step(state, dt),
            None => None,
        };
    }
}
