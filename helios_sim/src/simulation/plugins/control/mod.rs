// helios_sim/src/simulation/plugins/control/mod.rs
//
// ControlPlugin: initializes ControlOutputComponent at scene build time and
// each tick copies the AutonomyPipeline's latest ControlOutput into it so
// the vehicle adapter (Layer 2) can consume it.

use crate::{
    prelude::*,
    simulation::{
        core::{
            app_state::{AppState, SceneBuildSet, SimulationSet},
            components::{ControlOutputComponent, ControllerStateSource},
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
            publish_pipeline_control
                .in_set(SimulationSet::Control)
                .run_if(in_state(AppState::Running)),
        );
    }
}

fn spawn_control_output(
    mut commands: Commands,
    agent_query: Query<Entity, With<AutonomyPipelineComponent>>,
) {
    for entity in &agent_query {
        commands.entity(entity).insert((
            ControlOutputComponent(ControlOutput::BodyVelocity {
                linear: Vector3::zeros(),
                angular: Vector3::zeros(),
            }),
            // Required by the vehicle HUD and toggled with T at runtime.
            // GroundTruth is the safer default so the controller sees real
            // physics state until the estimator has spun up.
            ControllerStateSource::GroundTruth,
        ));
    }
}

/// Copies the pipeline's latest `ControlOutput` (written by the controller
/// node inside the DAG during `SimulationSet::Estimation`) into
/// `ControlOutputComponent` so the vehicle adapter in `SimulationSet::Actuation`
/// can read it.
fn publish_pipeline_control(
    mut query: Query<(&AutonomyPipelineComponent, &mut ControlOutputComponent)>,
) {
    let _span = tracing::trace_span!("sim.control.publish").entered();
    for (pipeline, mut output) in &mut query {
        if let Some(stamped) = pipeline.0.read_control() {
            output.0 = stamped.value.clone();
        }
    }
}
