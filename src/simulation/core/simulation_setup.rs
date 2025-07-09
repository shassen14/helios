// src/simulation/core/simulation_setup.rs

use crate::prelude::{AgentConfig, AppState, GroundTruthState};
use crate::simulation::core::app_state::SceneBuildSet;
use crate::simulation::core::config::SimulationConfig;
use crate::simulation::core::frames::MeasurementEvent;
use crate::simulation::core::prng::SimulationRng;
// use crate::simulation::core::registries::{EstimatorRegistry, , VehicleRegistry};
use crate::simulation::core::topics::TopicBus;
use bevy::prelude::*;
use rand::SeedableRng;
use rand::rngs::OsRng;
use rand_chacha::ChaCha8Rng;

#[derive(Component, Clone)] // It must derive Clone
pub struct SpawnAgentConfigRequest(pub AgentConfig);

pub struct SimulationSetupPlugin;

impl Plugin for SimulationSetupPlugin {
    fn build(&self, app: &mut App) {
        // This plugin's job is to read the config and add resources and startup systems.
        let config = app
            .world()
            .get_resource::<SimulationConfig>()
            .expect("SimulationConfig not found!");

        // --- 1. Add the Deterministic PRNG Resource ---
        let rng = match config.simulation.seed {
            Some(seed) => ChaCha8Rng::seed_from_u64(seed),
            None => ChaCha8Rng::from_rng(&mut OsRng).expect("OS RNG failed"),
        };
        app.insert_resource(SimulationRng(rng));
        app.init_resource::<TopicBus>();
        app.add_event::<MeasurementEvent>();

        app.configure_sets(
            OnEnter(AppState::SceneBuilding),
            (
                SceneBuildSet::CreateRequests,
                SceneBuildSet::ProcessVehicle,
                SceneBuildSet::ProcessSensors,
                SceneBuildSet::ProcessBaseAutonomy,
                SceneBuildSet::ProcessDependentAutonomy,
                SceneBuildSet::ProcessControllers,
                SceneBuildSet::Physics,
                SceneBuildSet::Validation,
                SceneBuildSet::Cleanup,
            )
                .chain(),
        );

        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            (
                // PASS 1: Create agent shells and post requests
                spawn_agent_shells.in_set(SceneBuildSet::CreateRequests),
                // PASS 2 (handled by plugins): Fulfill requests
                // process_autonomy_requests.in_set(SceneBuildSet::ProcessRequests),
                // PASS 3 (handled by plugins): Validate
                // PASS 4: Clean up the request components
                cleanup_spawn_requests.in_set(SceneBuildSet::Cleanup),
                // FINAL: Transition to the main simulation loop
                transition_to_running_state.after(SceneBuildSet::Cleanup),
            ),
        );
    }
}

fn spawn_agent_shells(mut commands: Commands, config: Res<SimulationConfig>) {
    for agent_config in &config.agents {
        info!(
            "[SPAWN] Posting full spawn request for agent: {}",
            &agent_config.name
        );

        // Convert the starting pose to both the logical Isometry and the Bevy Transform.
        let starting_pose = &agent_config.starting_pose;
        let start_isometry = starting_pose.to_isometry();
        let start_transform = starting_pose.to_bevy_transform(); // The Bevy Transform

        // Spawn the shell with its core state AND its initial transform.
        commands.spawn((
            Name::new(agent_config.name.clone()),
            GroundTruthState {
                pose: start_isometry,
                ..default()
            },
            // --- THE FIX for B0004 ---
            // Add the transform here. This ensures the parent always has a transform
            // before any children are added in later systems.
            start_transform,
            // The single request component
            SpawnAgentConfigRequest(agent_config.clone()),
        ));
    }
}
fn cleanup_spawn_requests(
    mut commands: Commands,
    query: Query<Entity, Or<(With<SpawnAgentConfigRequest>,)>>,
) {
    info!("[CLEANUP] Removing spawn request components.");
    for entity in &query {
        commands.entity(entity).remove::<SpawnAgentConfigRequest>();
    }
}

/// This simple system runs once at the end of the `OnEnter(SceneBuilding)` chain.
/// Its only job is to move the app into the main `Running` state.
fn transition_to_running_state(mut next_state: ResMut<NextState<AppState>>) {
    info!("Scene building complete. Transitioning to Running state.");
    next_state.set(AppState::Running);
}
