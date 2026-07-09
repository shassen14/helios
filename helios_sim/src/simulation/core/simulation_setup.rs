use std::time::Duration;

use avian3d::prelude::PhysicsSystems;
use rand::rngs::OsRng;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use super::components::GroundTruthState;
use super::transforms::{tf_tree_incremental_update_system, tf_tree_structural_system, TfTree};
use crate::prelude::*;
use crate::simulation::core::app_state::{AssetLoadSet, SimulationSet};
use crate::simulation::core::components::ConfiguredMissionGoal;
use crate::simulation::core::ground_truth::publish_oracle_channels_system;
use crate::simulation::core::ground_truth_sync_system;
use crate::simulation::core::prng::SimulationRng;
use crate::simulation::core::transforms::build_static_tf_maps;

pub struct SimulationSetupPlugin;

impl Plugin for SimulationSetupPlugin {
    fn build(&self, app: &mut App) {
        // --- INITIALIZE RESOURCES & EVENTS ---
        app.init_resource::<TfTree>();

        app.configure_sets(
            OnEnter(AppState::AssetLoading),
            (AssetLoadSet::Config, AssetLoadSet::Kickoff).chain(), // .chain() guarantees Config runs before Kickoff
        );

        app.add_systems(
            OnEnter(AppState::AssetLoading),
            initialize_simulation_resources.in_set(AssetLoadSet::Kickoff),
        );

        // --- CONFIGURE THE SPAWNING PIPELINE ---
        configure_scene_build_sets(app);

        configure_fixed_update_sets(app);

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

        // TF tree update strategy:
        // - Structural system (Precomputation): inserts/removes entries when TrackedFrame is
        //   added or removed. No-op on the vast majority of ticks.
        // - Incremental system (Precomputation): updates poses for Changed<GlobalTransform>.
        //   Handles first-tick initialization; no-op every tick after that until physics runs.
        // - Incremental system (StateSync): updates poses after Avian3D physics, so
        //   oracle publishing and any downstream consumers see the latest state.
        app.add_systems(
            FixedUpdate,
            (
                (tf_tree_structural_system, tf_tree_incremental_update_system)
                    .chain()
                    .in_set(SimulationSet::Precomputation)
                    .run_if(in_state(AppState::Running)),
                ground_truth_sync_system.in_set(SimulationSet::StateSync),
                publish_oracle_channels_system
                    .after(ground_truth_sync_system)
                    .in_set(SimulationSet::StateSync)
                    .run_if(in_state(AppState::Running)),
                tf_tree_incremental_update_system
                    .in_set(SimulationSet::StateSync)
                    .after(ground_truth_sync_system)
                    .run_if(in_state(AppState::Running)),
            ),
        );
    }
}

// Chain the scene-building sets so each pass runs after the previous one's
// spawns are visible. Every set boundary is a Commands sync point, so a pass
// that queries the prior pass's inserts (e.g. `SpawnPipeline` reading the
// sensor channels from `ProcessSensors`) must sit in a later set, not just a
// later system.
fn configure_scene_build_sets(app: &mut App) {
    app.configure_sets(
        OnEnter(AppState::SceneBuilding),
        (
            SceneBuildSet::CreateRequests,
            SceneBuildSet::ProcessWorldObjects,
            SceneBuildSet::ProcessVehicle,
            SceneBuildSet::ProcessSensors,
            SceneBuildSet::SpawnPipeline,
            SceneBuildSet::BindPipeline,
            SceneBuildSet::Physics,
            SceneBuildSet::Finalize,
            SceneBuildSet::Cleanup,
        )
            .chain(),
    );
}

fn configure_fixed_update_sets(app: &mut App) {
    // Configure the runtime schedule graph.
    app.configure_sets(
        FixedUpdate,
        (
            // Prepare TF from current transforms.
            SimulationSet::Precomputation,
            // Simulate sensors, publish readings to the bus.
            SimulationSet::Sensors,
            // Tick the whole autonomy pipeline (every DAG node).
            SimulationSet::BrainTick,
            // Feed goals into the pipeline, then drain its outputs to ECS.
            SimulationSet::BrainInput,
            SimulationSet::BrainOutput,
            // Convert control to physical forces.
            SimulationSet::Actuation,
            // Physics, then read state back.
            (
                // Avian's internal set where it prepares bodies.
                PhysicsSystems::Prepare,
                // The main physics simulation step.
                PhysicsSystems::StepSimulation,
                // Our system runs IMMEDIATELY AFTER the simulation.
                SimulationSet::StateSync,
            ),
        )
            .chain(), // .chain() enforces the order of the tuples/sets
    );
}

// Seed the simulation RNG and set the fixed-update rate from the resolved
// scenario config. This runs in `AssetLoadSet::Kickoff`, after the config
// loader has populated `ScenarioConfig` in `AssetLoadSet::Config`. Reading these
// values any earlier (as in `Plugin::build`) sees only the struct defaults, not
// the loaded TOML — so a configured seed would never reach the RNG.
fn initialize_simulation_resources(mut commands: Commands, config: Res<ScenarioConfig>) {
    // A configured seed makes the run reproducible: the same seed always yields
    // the same sequence. Without one, fall back to OS entropy, which differs
    // from run to run and is therefore non-deterministic.
    let rng = match config.simulation.seed {
        Some(seed) => ChaCha8Rng::seed_from_u64(seed),
        None => ChaCha8Rng::from_rng(&mut OsRng).expect("OS RNG failed"),
    };

    commands.insert_resource(SimulationRng(rng));

    // Drive FixedUpdate at the configured rate: one tick every `1 / frequency`
    // seconds.
    let frequency_hz = config.simulation.frequency_hz;
    commands.insert_resource(Time::<Fixed>::from_duration(Duration::from_secs_f64(
        1.0 / frequency_hz,
    )));
}

fn spawn_agent_shells(mut commands: Commands, config: Res<ScenarioConfig>) {
    // The resolution and deserialization logic has been moved to the config module.
    // We can now simply iterate over the final, concrete `AgentConfig` structs.
    for agent_config in &config.agents {
        info!(
            "[SPAWN] Posting spawn request for resolved agent: {}",
            agent_config.name()
        );

        // This part of your logic remains the same.
        let starting_pose = &agent_config.starting_pose;
        let start_isometry = starting_pose.to_isometry();
        let start_transform = starting_pose.to_bevy_transform();

        commands.spawn((
            Name::new(format!("{}/base_link", agent_config.name())),
            TrackedFrame,
            GroundTruthState {
                pose: start_isometry,
                ..default()
            },
            start_transform,
            // We clone the agent_config to move ownership into the component.
            // This is necessary because multiple systems in your spawning pipeline
            // (for sensors, estimators, etc.) will need to read from it.
            SpawnAgentConfigRequest(agent_config.clone()),
            ConfiguredMissionGoal(PlannerGoal::WorldPose(agent_config.goal_pose.to_isometry())),
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
fn transition_to_running(mut next_state: ResMut<NextState<AppState>>) {
    info!("Scene building complete. Transitioning to Running state.");
    next_state.set(AppState::Running);
}

#[cfg(test)]
mod tests {
    use super::{configure_fixed_update_sets, configure_scene_build_sets};
    use crate::simulation::core::app_state::{AppState, SceneBuildSet, SimulationSet};

    use bevy::prelude::*;

    // Records the order in which the FixedUpdate sim sets actually fire.
    // Every probe writes this resource, so Bevy must serialize them — the only
    // execution order that satisfies the set graph is the one we assert.
    #[derive(Resource, Default)]
    struct SetOrder(Vec<&'static str>);

    #[test]
    fn fixed_update_sets_run_in_declared_order() {
        let mut app = App::new();
        app.init_resource::<SetOrder>();

        // The real ordering under test — same function the plugin calls.
        configure_fixed_update_sets(&mut app);

        // One trivial probe per set: append the set's name, nothing else.
        app.add_systems(
            FixedUpdate,
            (
                (|mut o: ResMut<SetOrder>| o.0.push("Precomputation"))
                    .in_set(SimulationSet::Precomputation),
                (|mut o: ResMut<SetOrder>| o.0.push("Sensors")).in_set(SimulationSet::Sensors),
                (|mut o: ResMut<SetOrder>| o.0.push("BrainTick")).in_set(SimulationSet::BrainTick),
                (|mut o: ResMut<SetOrder>| o.0.push("BrainInput"))
                    .in_set(SimulationSet::BrainInput),
                (|mut o: ResMut<SetOrder>| o.0.push("BrainOutput"))
                    .in_set(SimulationSet::BrainOutput),
                (|mut o: ResMut<SetOrder>| o.0.push("Actuation")).in_set(SimulationSet::Actuation),
                (|mut o: ResMut<SetOrder>| o.0.push("StateSync")).in_set(SimulationSet::StateSync),
            ),
        );

        // Run the schedule exactly once — bypasses the Time<Fixed> accumulator,
        // so we get one deterministic pass instead of N wall-clock-driven steps.
        app.world_mut().run_schedule(FixedUpdate);

        assert_eq!(
            app.world().resource::<SetOrder>().0,
            vec![
                "Precomputation",
                "Sensors",
                "BrainTick",
                "BrainInput",
                "BrainOutput",
                "Actuation",
                "StateSync",
            ],
        );
    }

    #[test]
    fn scene_build_sets_run_in_declared_order() {
        let mut app = App::new();
        app.init_resource::<SetOrder>();

        // The real ordering under test — same function the plugin calls.
        configure_scene_build_sets(&mut app);

        // One trivial probe per set: append the set's name, nothing else.
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            (
                (|mut o: ResMut<SetOrder>| o.0.push("CreateRequests"))
                    .in_set(SceneBuildSet::CreateRequests),
                (|mut o: ResMut<SetOrder>| o.0.push("ProcessWorldObjects"))
                    .in_set(SceneBuildSet::ProcessWorldObjects),
                (|mut o: ResMut<SetOrder>| o.0.push("ProcessVehicle"))
                    .in_set(SceneBuildSet::ProcessVehicle),
                (|mut o: ResMut<SetOrder>| o.0.push("ProcessSensors"))
                    .in_set(SceneBuildSet::ProcessSensors),
                (|mut o: ResMut<SetOrder>| o.0.push("SpawnPipeline"))
                    .in_set(SceneBuildSet::SpawnPipeline),
                (|mut o: ResMut<SetOrder>| o.0.push("BindPipeline"))
                    .in_set(SceneBuildSet::BindPipeline),
                (|mut o: ResMut<SetOrder>| o.0.push("Physics")).in_set(SceneBuildSet::Physics),
                (|mut o: ResMut<SetOrder>| o.0.push("Finalize")).in_set(SceneBuildSet::Finalize),
                (|mut o: ResMut<SetOrder>| o.0.push("Cleanup")).in_set(SceneBuildSet::Cleanup),
            ),
        );

        // Run the OnEnter(SceneBuilding) schedule directly — no state machine
        // needed, we just fire the schedule the sets are configured against.
        app.world_mut()
            .run_schedule(OnEnter(AppState::SceneBuilding));

        assert_eq!(
            app.world().resource::<SetOrder>().0,
            vec![
                "CreateRequests",
                "ProcessWorldObjects",
                "ProcessVehicle",
                "ProcessSensors",
                "SpawnPipeline",
                "BindPipeline",
                "Physics",
                "Finalize",
                "Cleanup",
            ],
        );
    }
}
