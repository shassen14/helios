// helios_sim/src/simulation/plugins/control/mod.rs
//
// ControlPlugin: initializes ControlOutputComponent at scene build time.
// Step 9b will wire this to AutonomyPipelineComponent for real control output.

use crate::{
    prelude::*,
    simulation::{
        core::{
            app_state::{AppState, SceneBuildSet, SimulationSet},
            components::ControlOutputComponent,
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
            controller_compute_stub.in_set(SimulationSet::Control),
        );
    }
}

fn spawn_control_output(
    mut commands: Commands,
    agent_query: Query<Entity, With<AutonomyPipelineComponent>>,
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

/// No-op stub. Control will be wired to the pipeline in a later step.
fn controller_compute_stub() {}
