// helios_bevy_sim/src/simulation/core/simulation_setup.rs

use crate::prelude::*;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::config::SimulationConfig;
use crate::simulation::core::events::BevyMeasurementEvent;
// Import the Bevy-specific types this plugin manages
use super::topics::{GroundTruthState, TopicBus};
use super::transforms::{tf_tree_builder_system, TfTree};

pub struct SimulationSetupPlugin;

impl Plugin for SimulationSetupPlugin {
    fn build(&self, app: &mut App) {
        // --- INITIALIZE RESOURCES & EVENTS ---
        app
            // This resource is the central message bus for the simulation.
            .init_resource::<TopicBus>()
            // This resource will be populated each frame with the latest transforms.
            .init_resource::<TfTree>()
            // This is CRITICAL. It registers the pure MeasurementEvent with Bevy's event system.
            .add_event::<BevyMeasurementEvent>();

        // --- CONFIGURE THE SPAWNING PIPELINE ---
        // This chain of SystemSets guarantees the correct spawning order.
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

        // --- ADD THE CORE SYSTEMS TO THE SCHEDULE ---
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            (
                // This system reads the config and creates entities with "request" components.
                spawn_agent_shells.in_set(SceneBuildSet::CreateRequests),
                // This system removes the temporary request components after all processing is done.
                cleanup_spawn_requests.in_set(SceneBuildSet::Cleanup),
                // This system transitions to the main simulation loop after building is complete.
                transition_to_running_state.after(SceneBuildSet::Cleanup),
            ),
        );

        // Configure the runtime schedule graph.
        app.configure_sets(
            FixedUpdate,
            (
                // Phase 1
                SimulationSet::Precomputation,
                // Phase 2 (these two can run in parallel with each other)
                (SimulationSet::Sensors, SimulationSet::Perception),
                // Phase 3
                SimulationSet::WorldModeling,
                // Phase 4
                (SimulationSet::Estimation, SimulationSet::Behavior),
                // Phase 5
                SimulationSet::Planning,
                SimulationSet::Control,
                // Phase 6
                SimulationSet::Actuation,
            )
                .chain(), // .chain() enforces the order of the tuples/sets
        );

        // Define the more complex dependencies
        app.configure_sets(
            FixedUpdate,
            (
                // WorldModeling must run after BOTH Sensors and Perception are complete.
                SimulationSet::WorldModeling
                    .after(SimulationSet::Sensors)
                    .after(SimulationSet::Perception),
                // Estimation needs the latest sensor data.
                SimulationSet::Estimation.after(SimulationSet::Sensors),
                // Behavior needs the latest state estimate and world model.
                SimulationSet::Behavior
                    .after(SimulationSet::Estimation)
                    .after(SimulationSet::WorldModeling),
                // Planning is triggered by Behavior.
                SimulationSet::Planning.after(SimulationSet::Behavior),
                // Control is triggered by Planning.
                SimulationSet::Control.after(SimulationSet::Planning),
                // Actuation is triggered by Control.
                SimulationSet::Actuation.after(SimulationSet::Control),
            ),
        );

        // Add the TF tree builder to the main game loop.
        // It must run before any system that needs to query for transforms, like the EKF.
        app.add_systems(
            FixedUpdate,
            tf_tree_builder_system
                .in_set(SimulationSet::Precomputation)
                .run_if(in_state(AppState::Running)),
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
