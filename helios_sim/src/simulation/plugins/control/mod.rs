// helios_sim/src/simulation/plugins/control/mod.rs
//
// ControlPlugin: initializes ControlOutputComponent at scene build time and calls
// step_controllers() each tick via EstimatorComponent (for state) + ControlPipelineComponent.

use crate::{
    prelude::*,
    simulation::{
        core::{
            app_state::{AppState, SceneBuildSet, SimulationSet},
            components::{ControlOutputComponent, ControllerStateSource, GroundTruthState},
            sim_runtime::SimRuntime,
        },
        plugins::autonomy::{
            ControlPipelineComponent, EstimatorComponent, PathFollowingOutputComponent,
        },
    },
};

use helios_core::control::ControlOutput;
use helios_core::frames::FrameAwareState;
use helios_core::types::FrameHandle;
use nalgebra::Vector3;

pub struct ControlPlugin;

impl Plugin for ControlPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_control_output.in_set(SceneBuildSet::ProcessControllers),
        )
        .add_systems(
            FixedUpdate,
            controller_compute_system.in_set(SimulationSet::Control),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

/// Inserts `ControlOutputComponent` on every agent that has an `EstimatorComponent`.
fn spawn_control_output(
    mut commands: Commands,
    agent_query: Query<Entity, With<EstimatorComponent>>,
) {
    for entity in &agent_query {
        commands
            .entity(entity)
            .insert(ControlOutputComponent(ControlOutput::BodyVelocity {
                linear: Vector3::zeros(),
                angular: Vector3::zeros(),
            }));
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

/// Calls `step_controllers()` each tick.
/// Reads state from `EstimatorComponent` or `GroundTruthState` based on `ControllerStateSource`.
fn controller_compute_system(
    time: Res<Time>,
    tf_tree: Res<crate::simulation::core::transforms::TfTree>,
    mut query: Query<(
        Entity,
        &EstimatorComponent,
        &mut ControlPipelineComponent,
        &mut ControlOutputComponent,
        &ControllerStateSource,
        Option<&GroundTruthState>,
        &PathFollowingOutputComponent,
    )>,
) {
    let dt = time.delta_secs_f64();
    let ts = time.elapsed_secs_f64();
    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: ts,
    };

    for (entity, estimator, mut control, mut output, state_source, gt_opt, pf_output) in &mut query
    {
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

        if let Some(state) = state {
            if let Some(out) = control
                .0
                .step_controllers(state, pf_output.0.as_ref(), dt, &runtime)
            {
                output.0 = out;
            }
        }
    }
}
