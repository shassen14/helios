// helios_sim/src/simulation/plugins/control/mod.rs
//
// ControlPlugin: spawns ControllerComponent from TOML config, runs controller compute
// each tick, and writes ControlOutputComponent for the Actuation set to consume.

use crate::{
    prelude::*,
    simulation::{
        core::{
            app_state::{AppState, SceneBuildSet, SimulationSet},
            components::{ControllerComponent, ControlOutputComponent},
        },
        plugins::world_model::WorldModelComponent,
        registry::{AutonomyRegistry, ControllerBuildContext},
    },
};

use helios_core::control::{ControlContext, ControlOutput};
use nalgebra::Vector3;

pub struct ControlPlugin;

impl Plugin for ControlPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_controllers.in_set(SceneBuildSet::ProcessControllers),
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

/// Reads each agent's `AutonomyStack.controllers` config and inserts
/// `ControllerComponent` on the entity using the `AutonomyRegistry`.
fn spawn_controllers(
    mut commands: Commands,
    registry: Res<AutonomyRegistry>,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (entity, request) in &request_query {
        let cfg = &request.0;
        let controllers = &cfg.autonomy_stack.controllers;

        if controllers.is_empty() {
            continue;
        }

        // For now we pick the first controller. Multi-controller routing can be extended later.
        let Some((key, ctrl_cfg)) = controllers.iter().next() else {
            continue;
        };

        let kind = match ctrl_cfg {
            crate::simulation::config::structs::ControllerConfig::Pid { .. } => "Pid",
            crate::simulation::config::structs::ControllerConfig::Lqr { .. } => "Lqr",
            crate::simulation::config::structs::ControllerConfig::FeedforwardPid { .. } => {
                "FeedforwardPid"
            }
        };

        let ctx = ControllerBuildContext {
            agent_entity: entity,
            controller_cfg: ctrl_cfg.clone(),
            agent_config: cfg.clone(),
            dynamics_factories: registry.clone_dynamics_arc(),
        };

        match registry.build_controller(kind, ctx) {
            Ok(ctrl) => {
                commands
                    .entity(entity)
                    .insert(ControllerComponent(ctrl))
                    .insert(ControlOutputComponent(ControlOutput::BodyVelocity {
                        linear: Vector3::zeros(),
                        angular: Vector3::zeros(),
                    }));
                info!(
                    "Spawned controller '{}' (key '{}') on entity {:?}",
                    kind, key, entity
                );
            }
            Err(e) => {
                error!(
                    "Failed to build controller '{}' for entity {:?}: {}",
                    kind, entity, e
                );
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

/// Calls `Controller::compute()` each tick for every entity that has both
/// a `ControllerComponent` and a `WorldModelComponent` (state estimate available).
/// Mutates `ControlOutputComponent` in-place — no deferred commands on the hot path.
fn controller_compute_system(
    time: Res<Time>,
    mut query: Query<(
        &mut ControllerComponent,
        &mut ControlOutputComponent,
        Option<&WorldModelComponent>,
    )>,
) {
    let dt = time.delta_secs_f64();

    for (mut ctrl_comp, mut output_comp, world_model_opt) in &mut query {
        // Require an estimated state — skip if the world model isn't ready yet.
        let Some(world_model) = world_model_opt else {
            continue;
        };

        let state = match world_model {
            WorldModelComponent::Separate { estimator, .. } => estimator.get_state(),
            WorldModelComponent::CombinedSlam { system } => system.get_state(),
        };

        let ctx = ControlContext {
            tf: None,
            reference: None,
        };

        output_comp.0 = ctrl_comp.0.compute(state, dt, &ctx);
    }
}
