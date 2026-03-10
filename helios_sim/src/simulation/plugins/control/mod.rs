// helios_sim/src/simulation/plugins/control/mod.rs
//
// ControlPlugin: initializes ControlOutputComponent at scene build time and calls
// step_controllers() each tick via the pipeline stored in AutonomyPipelineComponent.

use crate::{
    prelude::*,
    simulation::{
        core::{
            app_state::{AppState, SceneBuildSet, SimulationSet},
            components::ControlOutputComponent,
            sim_runtime::SimRuntime,
        },
        plugins::autonomy::AutonomyPipelineComponent,
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

/// Inserts `ControlOutputComponent` on every agent that has an `AutonomyPipelineComponent`.
/// The pipeline's controllers (if any) will write into this component each tick.
fn spawn_control_output(
    mut commands: Commands,
    agent_query: Query<Entity, With<AutonomyPipelineComponent>>,
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

/// Calls `step_controllers()` each tick for every agent that has both
/// an `AutonomyPipelineComponent` and a `ControlOutputComponent`.
/// Mutates `ControlOutputComponent` in-place — no deferred commands on the hot path.
fn controller_compute_system(
    time: Res<Time>,
    tf_tree: Res<crate::simulation::core::transforms::TfTree>,
    mut query: Query<(&mut AutonomyPipelineComponent, &mut ControlOutputComponent)>,
) {
    let dt = time.delta_secs_f64();
    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    for (mut pipeline_comp, mut output_comp) in &mut query {
        if let Some(output) = pipeline_comp.0.step_controllers(dt, &runtime) {
            output_comp.0 = output;
        }
    }
}
