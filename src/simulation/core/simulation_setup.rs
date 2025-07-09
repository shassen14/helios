// src/simulation/core/simulation_setup.rs

use crate::prelude::{
    AgentConfig, AppState, AutonomyStack, GroundTruthState, SensorConfig, Vehicle,
};
use crate::simulation::core::app_state::SceneBuildSet;
use crate::simulation::core::config::SimulationConfig;
use crate::simulation::core::prng::SimulationRng;
// use crate::simulation::core::registries::{EstimatorRegistry, , VehicleRegistry};
use crate::simulation::core::topics::TopicBus;
use bevy::prelude::*;
use rand::SeedableRng;
use rand::rngs::OsRng;
use rand_chacha::ChaCha8Rng;

#[derive(Component)]
pub struct SpawnRequestVehicle(pub Vehicle);

#[derive(Component)]
pub struct SpawnRequestSensorSuite(pub Vec<SensorConfig>);

#[derive(Component)]
pub struct SpawnRequestAutonomy(pub AutonomyStack);
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

        app.configure_sets(
            OnEnter(AppState::SceneBuilding),
            (
                SceneBuildSet::CreateRequests,
                SceneBuildSet::ProcessRequests,
                SceneBuildSet::Physics,
                SceneBuildSet::Validation,
                SceneBuildSet::Cleanup, // Add a cleanup set
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

        // app.configure_sets(
        //     OnEnter(AppState::SceneBuilding),
        //     (
        //         SceneBuildSet::Logic,
        //         SceneBuildSet::Physics,
        //         SceneBuildSet::Validation,
        //     )
        //         .chain(), // .chain() enforces the sequence.
        // );

        // // --- 3. Add the startup systems that depend on the config ---
        // app.add_systems(
        //     OnEnter(AppState::SceneBuilding),
        //     (
        //         // --- Add Logic Set Systems ---
        //         (spawn_agents_from_config).in_set(SceneBuildSet::Logic),
        //         // --- Add Validation Set Systems ---
        //         // We'll create a placeholder for this system for now.
        //         validate_agent_pipelines.in_set(SceneBuildSet::Validation),
        //         // --- Add the final transition system ---
        //         // This will run after all the sets in the chain are complete.
        //         transition_to_running_state.after(SceneBuildSet::Validation),
        //     ),
        // );
    }
}

fn spawn_agent_shells(mut commands: Commands, config: Res<SimulationConfig>) {
    for agent_config in &config.agents {
        info!(
            "[SPAWN] Posting spawn requests for agent: {}",
            &agent_config.name
        );

        let start_isometry = agent_config.starting_pose.to_isometry();

        commands.spawn((
            // Core components
            Name::new(agent_config.name.clone()),
            GroundTruthState {
                pose: start_isometry,
                ..default()
            },
            // "Job" Request Components
            SpawnRequestVehicle(agent_config.vehicle.clone()),
            SpawnRequestSensorSuite(agent_config.sensors.clone()),
            SpawnRequestAutonomy(agent_config.autonomy_stack.clone()),
            SpawnAgentConfigRequest(agent_config.clone()),
        ));
    }
}

// --- SYSTEM 2: CLEANUP ---
fn cleanup_spawn_requests(
    mut commands: Commands,
    query: Query<
        Entity,
        Or<(
            With<SpawnRequestVehicle>,
            With<SpawnRequestSensorSuite>,
            With<SpawnRequestAutonomy>,
        )>,
    >,
) {
    info!("[CLEANUP] Removing spawn request components.");
    for entity in &query {
        commands.entity(entity).remove::<(
            SpawnRequestVehicle,
            SpawnRequestSensorSuite,
            SpawnRequestAutonomy,
        )>();
    }
}

// fn process_autonomy_requests(
//     mut commands: Commands,
//     topic_bus: Res<TopicBus>,
//     // Get all the autonomy registries
//     estimator_registry: Res<EstimatorRegistry>,
//     // mapper_registry: Res<MapperRegistry>, etc.
//     // Query for agents that have an autonomy request
//     request_query: Query<(Entity, &Name, &SpawnRequestAutonomy)>,
// ) {
//     for (entity, name, request) in &request_query {
//         let autonomy_config = &request.0;

//         // --- Process Estimator Slot ---
//         for estimator_config in &autonomy_config.estimators {
//             let type_str = estimator_config.get_type_str();
//             if let Some(spawner) = estimator_registry.0.get(type_str) {
//                 // The spawner needs to know the unique name to create unique topic names
//                 // e.g., /state/estimated/baseline
//                 spawner(&mut commands.entity(entity), name, estimator_config, ...);
//             }
//         }

//         // --- Process Mapper Slot (Future) ---
//         // if let Some(mapper_config) = &autonomy_config.mapper {
//         //     let type_str = mapper_config.get_type_str();
//         //     if let Some(spawner) = mapper_registry.0.get(type_str) {
//         //         spawner(...);
//         //     }
//         // }
//     }
// }

// fn spawn_agents_from_config(
//     mut commands: Commands,
//     config: Res<SimulationConfig>,
//     vehicle_registry: Res<VehicleRegistry>,
//     sensor_registry: Res<SensorRegistry>,
//     estimator_registry: Res<EstimatorRegistry>,
//     mut topic_bus: ResMut<TopicBus>,
// ) {
//     for agent_config in &config.agents {
//         info!("[SPAWN] Creating agent shell: {}", &agent_config.name);

//         // --- Calculate Initial Logical Pose ---
//         let start_pos = agent_config.starting_pose.position;
//         let start_rot_deg = agent_config.starting_pose.orientation_deg;
//         let start_isometry_enu = Isometry3::from_parts(
//             Translation3::new(
//                 start_pos[0] as f64,
//                 start_pos[1] as f64,
//                 start_pos[2] as f64,
//             ),
//             UnitQuaternion::from_euler_angles(
//                 start_rot_deg[0].to_radians() as f64,
//                 start_rot_deg[1].to_radians() as f64,
//                 start_rot_deg[2].to_radians() as f64,
//             ),
//         );

//         // --- Spawn the Logical Entity Shell ---
//         // This entity has a name and a logical state, but no physical form yet.
//         let mut agent_entity_commands = commands.spawn((
//             Name::new(agent_config.name.clone()),
//             GroundTruthState {
//                 pose: start_isometry_enu,
//                 linear_velocity: Vector3::zeros(),
//                 angular_velocity: Vector3::zeros(),
//                 linear_acceleration: Vector3::zeros(),
//                 last_linear_velocity: Vector3::zeros(),
//             },
//         ));

//         // --- 1. Dispatch Vehicle Spawning ---
//         let vehicle_type_str = agent_config.vehicle.get_type_str(); // You'll need to add this helper method
//         if let Some(spawner) = vehicle_registry.0.get(vehicle_type_str) {
//             info!(
//                 "  -> Found spawner for vehicle type '{}'. Spawning...",
//                 vehicle_type_str
//             );
//             // Call the registered spawner function from the AckermannPlugin
//             spawner(&mut agent_entity_commands, &agent_config.vehicle);
//         } else {
//             error!(
//                 "!-> No vehicle spawner registered for type '{}'",
//                 vehicle_type_str
//             );
//         }

//         // --- 2. Dispatch Sensor Spawning (for each sensor in the list) ---
//         for sensor_config in &agent_config.sensors {
//             let sensor_kind_str = sensor_config.get_kind_str();
//             if let Some(spawner) = sensor_registry.0.get(sensor_kind_str) {
//                 // Pass the agent_name and the mutable topic_bus to the spawner
//                 spawner(
//                     &agent_config.name,
//                     &mut agent_entity_commands,
//                     sensor_config,
//                     &mut topic_bus,
//                 );
//             }
//         }

//         // --- 3. Dispatch Estimator Spawning ---
//         if let Some(estimator_config) = &agent_config.autonomy_stack.estimator {
//             let estimator_type_str = estimator_config.get_type_str();
//             if let Some(spawner) = estimator_registry.0.get(estimator_type_str) {
//                 // Pass the agent_name and the immutable topic_bus
//                 spawner(
//                     &agent_config.name,
//                     &mut agent_entity_commands,
//                     estimator_config,
//                     &topic_bus,
//                 );
//             }
//         }
//     }
// }

/// This simple system runs once at the end of the `OnEnter(SceneBuilding)` chain.
/// Its only job is to move the app into the main `Running` state.
fn transition_to_running_state(mut next_state: ResMut<NextState<AppState>>) {
    info!("Scene building complete. Transitioning to Running state.");
    next_state.set(AppState::Running);
}

// fn validate_agent_pipelines() {
//     // Placeholder for your manifest validation logic.
//     info!("[VALIDATION] Checking agent pipeline configurations...");
//     // In the future, this system will query for LocalManifests and check for inconsistencies.
// }
