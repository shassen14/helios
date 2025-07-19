// helios_sim/src/simulation/core/simulation_setup.rs

use std::time::Duration;

use avian3d::prelude::PhysicsSet;
use rand::rngs::OsRng;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::prelude::*;
use crate::simulation::config::ResolvedAgents;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::ground_truth_sync_system;
use crate::simulation::core::prng::SimulationRng;
use crate::simulation::core::transforms::build_static_tf_maps;
// Import the Bevy-specific types this plugin manages
use super::topics::{GroundTruthState, TopicBus};
use super::transforms::{tf_tree_builder_system, TfTree};

pub struct SimulationSetupPlugin;

impl Plugin for SimulationSetupPlugin {
    fn build(&self, app: &mut App) {
        // This plugin's job is to read the config and add resources and startup systems.
        let config = app
            .world()
            .get_resource::<ScenarioConfig>()
            .expect("SimulationConfig not found!");

        // --- 1. Add the Deterministic PRNG Resource ---
        let rng = match config.simulation.seed {
            Some(seed) => ChaCha8Rng::seed_from_u64(seed),
            None => ChaCha8Rng::from_rng(&mut OsRng).expect("OS RNG failed"),
        };
        app.insert_resource(SimulationRng(rng));

        // --- INITIALIZE RESOURCES & EVENTS ---
        app
            // This resource is the central message bus for the simulation.
            .init_resource::<TopicBus>()
            // This resource will be populated each frame with the latest transforms.
            .init_resource::<TfTree>()
            // This is CRITICAL. It registers the pure MeasurementEvent with Bevy's event system.
            .add_event::<BevyMeasurementMessage>();

        let simulation_frequency = 400.0; // The desired frequency in Hz
        let fixed_update_timestep = 1.0 / simulation_frequency;

        app.insert_resource(
            // The resource is of type Time<Fixed>.
            Time::<Fixed>::from_duration(
                // We create a Duration from the calculated timestep.
                Duration::from_secs_f64(fixed_update_timestep),
            ),
        );

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
                SceneBuildSet::Finalize,
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
                build_static_tf_maps.in_set(SceneBuildSet::Finalize),
                // This system removes the temporary request components after all processing is done.
                cleanup_spawn_requests.in_set(SceneBuildSet::Cleanup),
                // This system transitions to the main simulation loop after building is complete.
                transition_to_running
                    .in_set(SceneBuildSet::Cleanup)
                    .after(cleanup_spawn_requests),
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
                // Phase 7
                (
                    // Avian's internal set where it prepares bodies.
                    PhysicsSet::Prepare,
                    // The main physics simulation step.
                    PhysicsSet::StepSimulation,
                    // Our system runs IMMEDIATELY AFTER the simulation.
                    SimulationSet::StateSync,
                ),
                SimulationSet::Validation,
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
            (
                tf_tree_builder_system
                    .in_set(SimulationSet::Precomputation)
                    .run_if(in_state(AppState::Running)),
                ground_truth_sync_system.in_set(SimulationSet::StateSync),
            ),
        );
    }
}

fn spawn_agent_shells(
    mut commands: Commands,
    // The system now depends on the `ResolvedAgents` resource.
    resolved_agents: Res<ResolvedAgents>,
) {
    // The resolution and deserialization logic has been moved to the config module.
    // We can now simply iterate over the final, concrete `AgentConfig` structs.
    for agent_config in &resolved_agents.0 {
        info!(
            "[SPAWN] Posting spawn request for resolved agent: {}",
            &agent_config.name
        );

        // This part of your logic remains the same.
        let starting_pose = &agent_config.starting_pose;
        let start_isometry = starting_pose.to_isometry();
        let start_transform = starting_pose.to_bevy_transform();

        commands.spawn((
            Name::new(agent_config.name.clone()),
            GroundTruthState {
                pose: start_isometry,
                ..default()
            },
            start_transform,
            // We clone the agent_config to move ownership into the component.
            // This is necessary because multiple systems in your spawning pipeline
            // (for sensors, estimators, etc.) will need to read from it.
            SpawnAgentConfigRequest(agent_config.clone()),
        ));
    }
}

// fn spawn_agent_shells(
//     mut commands: Commands,
//     config: Res<ScenarioConfig>,
//     catalog: Res<PrefabCatalog>,
// ) {
//     for agent_value in &config.agents {
//         info!("[SPAWN] Resolving agent configuration...");
//         let resolved_agent_value = match resolve_value(agent_value, &catalog) {
//             Ok(val) => val,
//             Err(e) => {
//                 error!("Failed to resolve agent config: {}", e);
//                 continue;
//             }
//         };

//         let agent_config: AgentConfig = match Value::deserialize(&resolved_agent_value) {
//             Ok(cfg) => cfg,
//             Err(e) => {
//                 // This error is super helpful for debugging bad TOML files.
//                 // It will tell you if a field is missing, has the wrong type, etc.
//                 error!(
//                     "Failed to deserialize resolved agent config into AgentConfig: {}",
//                     e
//                 );
//                 continue;
//             }
//         };

//         info!(
//             "[SPAWN] Posting spawn request for resolved agent: {}",
//             &agent_config.name
//         );

//         let starting_pose = &agent_config.starting_pose;
//         let start_isometry = starting_pose.to_isometry();
//         let start_transform = starting_pose.to_bevy_transform();

//         commands.spawn((
//             Name::new(agent_config.name.clone()),
//             GroundTruthState {
//                 pose: start_isometry,
//                 ..default()
//             },
//             start_transform,
//             SpawnAgentConfigRequest(agent_config),
//         ));
//     }
// }

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
fn transition_to_running(mut next_state: ResMut<NextState<AppState>>) {
    info!("Scene building complete. Transitioning to Running state.");
    next_state.set(AppState::Running);
}
