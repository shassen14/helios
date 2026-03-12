// helios_sim/src/simulation/plugins/control/mod.rs
//
// ControlPlugin: initializes ControlOutputComponent at scene build time and calls
// step_controllers() each tick via EstimatorComponent (for state) + ControlPipelineComponent.

use crate::{
    prelude::*,
    simulation::{
        core::{
            app_state::{AppState, SceneBuildSet, SimulationSet},
            components::ControlOutputComponent,
            sim_runtime::SimRuntime,
        },
        plugins::autonomy::{ControlPipelineComponent, EstimatorComponent},
    },
};

use helios_core::control::ControlOutput;
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
        commands.entity(entity).insert(ControlOutputComponent(ControlOutput::BodyVelocity {
            linear: Vector3::zeros(),
            angular: Vector3::zeros(),
        }));
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

/// Calls `step_controllers()` each tick.
/// Reads estimated state from `EstimatorComponent`, writes result to `ControlOutputComponent`.
fn controller_compute_system(
    time: Res<Time>,
    tf_tree: Res<crate::simulation::core::transforms::TfTree>,
    mut query: Query<(
        &EstimatorComponent,
        &mut ControlPipelineComponent,
        &mut ControlOutputComponent,
    )>,
) {
    let dt = time.delta_secs_f64();
    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    for (estimator, mut control, mut output) in &mut query {
        if let Some(state) = estimator.0.get_state() {
            if let Some(out) = control.0.step_controllers(state, dt, &runtime) {
                output.0 = out;
            }
        }
    }
}
