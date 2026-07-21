use std::time::Duration;

use avian3d::prelude::PhysicsSystems;
use bevy::time::TimeUpdateStrategy;
use rand::rngs::OsRng;
use rand::{RngCore, SeedableRng};
use rand_chacha::ChaCha8Rng;

use super::components::GroundTruthState;
use super::transforms::{tf_tree_incremental_update_system, tf_tree_structural_system, TfTree};
use crate::core::app_state::{AssetLoadSet, SimulationSet};
use crate::core::components::ConfiguredMissionGoal;
use crate::core::ground_truth::publish_oracle_channels_system;
use crate::core::ground_truth_sync_system;
use crate::core::host::TimePolicy;
use crate::core::prng::{MasterSeed, SimulationRng};
use crate::core::transforms::build_static_tf_maps;
use crate::prelude::*;

pub struct SimulationSetupPlugin;

impl Plugin for SimulationSetupPlugin {
    fn build(&self, app: &mut App) {
        // --- INITIALIZE RESOURCES & EVENTS ---
        app.init_resource::<TfTree>();

        app.configure_sets(
            OnEnter(AppState::AssetLoading),
            (AssetLoadSet::Config, AssetLoadSet::Kickoff).chain(), // .chain() guarantees Config runs before Kickoff
        );

        // Both read the loaded scenario and touch disjoint clocks, so they need
        // no ordering relative to each other.
        app.add_systems(
            OnEnter(AppState::AssetLoading),
            (initialize_simulation_resources, configure_time_pacing).in_set(AssetLoadSet::Kickoff),
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
            // Observe the fully-settled tick: ground truth and oracle channels
            // are fresh, so external consumers read the buses here.
            SimulationSet::Validation,
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
    // Resolve to a concrete seed *before* building any generator. An absent
    // config seed means "pick one," not "be unseeded": drawing a single u64 from
    // OS entropy leaves a value that can be logged and replayed, where seeding a
    // generator directly from entropy would leave nothing to quote in a bug
    // report. Every generator below is then built from this one number, which is
    // what makes the whole run reproducible from it alone.
    let (seed, source) = match config.simulation.seed {
        Some(s) => (s, "config"),
        None => (OsRng.next_u64(), "OS entropy"),
    };

    // Logged on every run, not just the drawn case: a bug report needs the seed
    // quoted whichever way it was chosen, and `source` is what tells the reader
    // whether re-running reproduces on its own or needs `--seed {seed}`.
    info!("master seed {seed} (from {source})");

    commands.insert_resource(MasterSeed(seed));
    commands.insert_resource(SimulationRng(ChaCha8Rng::seed_from_u64(seed)));

    // Drive FixedUpdate at the configured rate: one tick every `1 / frequency`
    // seconds.
    commands.insert_resource(Time::<Fixed>::from_duration(fixed_timestep(
        config.simulation.frequency_hz,
    )));
}

/// The `FixedUpdate` period implied by a tick rate.
///
/// Rejects a non-positive or non-finite rate rather than letting it reach
/// `Duration::from_secs_f64`, which would panic on the resulting infinity with
/// no mention of the config field responsible.
fn fixed_timestep(frequency_hz: f64) -> Duration {
    assert!(
        frequency_hz.is_finite() && frequency_hz > 0.0,
        "[simulation] frequency_hz must be a finite value greater than zero, got {frequency_hz}"
    );
    Duration::from_secs_f64(1.0 / frequency_hz)
}

/// Set up a fast-forwarded run's clock, which takes both a batch size and a
/// clamp wide enough to admit it.
///
/// Runs at kickoff rather than in `Plugin::build` because both quantities are
/// derived from the tick rate, and the scenario TOML that carries it isn't
/// loaded yet when the host is assembled.
fn configure_time_pacing(
    config: Res<ScenarioConfig>,
    policy: Res<TimePolicy>,
    mut virtual_time: ResMut<Time<Virtual>>,
    mut update_strategy: ResMut<TimeUpdateStrategy>,
) {
    if *policy != TimePolicy::FastAsPossible {
        return;
    }

    let step = fixed_timestep(config.simulation.frequency_hz);
    let steps_per_update = fast_forward_steps_per_update(step);

    // Takes the wall clock out of the loop: `time_system` synthesizes a delta
    // of `step * steps_per_update` per update instead of reading `Instant::now`,
    // so simulated time advances at whatever rate the machine can compute it.
    *update_strategy = TimeUpdateStrategy::FixedTimesteps(steps_per_update);

    // `Time<Virtual>` silently discards a delta above `max_delta` — a guard
    // meant for a stalled frame on a real clock. The delta here is synthetic,
    // so that guard can only do harm: the excess would vanish behind a `debug!`
    // and the run would simulate less time than the scenario asked for while
    // still reporting success.
    virtual_time.set_max_delta(fast_forward_max_delta(step, steps_per_update));
}

/// How much simulated time a fast-forwarded run may overshoot its stopping
/// point by.
///
/// Bevy's fixed loop runs a whole batch of steps per update and cannot be
/// broken out of partway, so a run that ends mid-batch still simulates the
/// remainder. The test harness pins its report to the deciding tick and stops
/// observing, but terminal assertions are judged against the final bus, which
/// those steps have touched — so this bounds how much extra motion can sit
/// behind the last value such an assertion reads.
///
/// A duration rather than a step count because the step count's meaning moves
/// with the tick rate: 25 steps is 60 ms at 400 Hz and 500 ms at 50 Hz. This is
/// the quantity that actually has to stay small, so it is the one pinned here
/// and the step count is derived from it.
const MAX_FAST_FORWARD_OVERSHOOT: Duration = Duration::from_millis(50);

/// Fixed steps to batch into one `App::update` while fast-forwarding.
///
/// Batching at all is what makes fast-forward worth having: a step is far
/// cheaper than the `App::update` wrapped around it, so stepping one at a time
/// spends most of a run on per-frame overhead. Batching measured several times
/// faster end to end.
///
/// The batch is capped by [`MAX_FAST_FORWARD_OVERSHOOT`] rather than chosen, so
/// a slower tick rate yields a smaller batch and the overshoot stays put.
/// Reproducibility doesn't constrain the choice — the loop runs exactly this
/// many steps every update however fast the machine is, so any value is equally
/// deterministic.
fn fast_forward_steps_per_update(timestep: Duration) -> u32 {
    // A tick rate slower than the whole overshoot budget can't batch at all:
    // one step already spends it. Never zero — that would stop the clock.
    let steps = MAX_FAST_FORWARD_OVERSHOOT.div_duration_f64(timestep) as u32;
    steps.max(1)
}

/// The clamp budget a fast-forwarded run needs: the synthetic delta it actually
/// feeds the clock, with room to spare so the two are never compared at the
/// boundary.
fn fast_forward_max_delta(timestep: Duration, steps_per_update: u32) -> Duration {
    // The clamp triggers on `>`, so an exact match would technically pass;
    // doubling keeps rounding in the step-count division off the edge.
    const HEADROOM: u32 = 2;

    timestep * steps_per_update * HEADROOM
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
    use super::{
        configure_fixed_update_sets, configure_scene_build_sets, configure_time_pacing,
        fast_forward_max_delta, fast_forward_steps_per_update, fixed_timestep,
        initialize_simulation_resources, MAX_FAST_FORWARD_OVERSHOOT,
    };
    use crate::core::app_state::{AppState, SceneBuildSet, SimulationSet};
    use crate::core::host::TimePolicy;
    use crate::core::prng::{MasterSeed, SimulationRng};
    use crate::prelude::ScenarioConfig;

    use std::time::Duration;

    use bevy::prelude::*;
    use bevy::time::TimeUpdateStrategy;
    use rand::{RngCore, SeedableRng};
    use rand_chacha::ChaCha8Rng;

    /// Slow enough that one fixed step alone spends the whole overshoot budget
    /// *and* exceeds Bevy's 250 ms default clamp — the rate at which batching
    /// must stop and an unguarded fast-forward would start losing time.
    const SLOWER_THAN_DEFAULT_CLAMP_HZ: f64 = 2.0;

    // A world holding just what `configure_time_pacing` reads and writes.
    fn pacing_world(frequency_hz: f64, policy: TimePolicy) -> World {
        let mut world = World::new();
        let mut config = ScenarioConfig::default();
        config.simulation.frequency_hz = frequency_hz;
        world.insert_resource(config);
        world.insert_resource(policy);
        world.insert_resource(Time::<Virtual>::default());
        world.insert_resource(TimeUpdateStrategy::Automatic);
        world
    }

    fn paced_world(frequency_hz: f64, policy: TimePolicy) -> World {
        let mut world = pacing_world(frequency_hz, policy);
        world
            .run_system_cached(configure_time_pacing)
            .expect("configure_time_pacing runs against the world it declares");
        world
    }

    fn max_delta_after_pacing(frequency_hz: f64, policy: TimePolicy) -> Duration {
        paced_world(frequency_hz, policy)
            .resource::<Time<Virtual>>()
            .max_delta()
    }

    #[test]
    fn the_timestep_is_the_reciprocal_of_the_tick_rate() {
        assert_eq!(fixed_timestep(400.0), Duration::from_secs_f64(0.0025));
    }

    #[test]
    #[should_panic(expected = "frequency_hz")]
    fn a_zero_tick_rate_is_refused_by_name() {
        // Left to `Duration::from_secs_f64`, this panics on an infinity without
        // naming the config field that produced it.
        fixed_timestep(0.0);
    }

    #[test]
    fn the_batch_never_outruns_the_overshoot_budget() {
        // The property the batch size exists to hold, checked across rates
        // rather than at one: a run can only overshoot by the steps left in the
        // batch after the deciding one, and that must stay inside the budget.
        for frequency_hz in [1.0, 50.0, 400.0, 10_000.0] {
            let step = fixed_timestep(frequency_hz);
            let overshoot = step * (fast_forward_steps_per_update(step) - 1);

            assert!(
                overshoot <= MAX_FAST_FORWARD_OVERSHOOT,
                "{frequency_hz} Hz overshoots by {overshoot:?}"
            );
        }
    }

    #[test]
    fn a_slower_tick_rate_earns_a_smaller_batch() {
        // The budget is a duration, so the step count has to fall as each step
        // grows — the point of deriving it instead of pinning a step count that
        // silently means eight times more overshoot at an eighth the rate.
        let fast = fast_forward_steps_per_update(fixed_timestep(400.0));
        let slow = fast_forward_steps_per_update(fixed_timestep(50.0));

        assert!(fast > slow, "400 Hz batched {fast}, 50 Hz batched {slow}");
    }

    #[test]
    fn a_tick_slower_than_the_whole_budget_still_batches_one_step() {
        // Zero would stop the clock outright, so the floor is one step even
        // when a single step already spends the entire budget.
        let step = fixed_timestep(SLOWER_THAN_DEFAULT_CLAMP_HZ);
        assert!(step > MAX_FAST_FORWARD_OVERSHOOT);

        assert_eq!(fast_forward_steps_per_update(step), 1);
    }

    // A world holding just what `initialize_simulation_resources` reads, with
    // the seed request under test. `seed: None` is the "pick one for me" case.
    fn seeded_world(seed: Option<u64>) -> World {
        let mut world = World::new();
        let mut config = ScenarioConfig::default();
        config.simulation.seed = seed;
        world.insert_resource(config);
        world
            .run_system_cached(initialize_simulation_resources)
            .expect("initialize_simulation_resources runs against the world it declares");
        world
    }

    #[test]
    fn a_configured_seed_reaches_the_master_seed_resource() {
        // The regression guard for the class of bug where a configured seed is
        // read too early and silently lost: this system runs in
        // `AssetLoadSet::Kickoff` precisely so the loaded TOML is visible, and
        // nothing else in the crate proves the value survives the trip.
        assert_eq!(seeded_world(Some(2024)).resource::<MasterSeed>().0, 2024);
    }

    #[test]
    fn an_absent_config_seed_is_still_resolved_and_recorded() {
        // An unseeded run must still end up with a concrete master seed — that
        // is what makes it replayable after the fact. The drawn value is
        // entropy, so its presence is the whole assertion available here.
        seeded_world(None).resource::<MasterSeed>();
    }

    #[test]
    fn the_simulation_rng_is_derived_from_the_master_seed() {
        // Replay depends on every generator descending from the recorded seed.
        // An RNG seeded from any other source would still produce a working
        // run, and the log would name a seed that reproduces none of it.
        let mut world = seeded_world(None);
        let master = world.resource::<MasterSeed>().0;

        let mut expected = ChaCha8Rng::seed_from_u64(master);
        let actual = world.resource_mut::<SimulationRng>().0.next_u64();

        assert_eq!(actual, expected.next_u64());
    }

    #[test]
    fn fast_forward_detaches_the_clock_from_the_wall() {
        let world = paced_world(400.0, TimePolicy::FastAsPossible);

        let step = fixed_timestep(400.0);
        assert!(matches!(
            world.resource::<TimeUpdateStrategy>(),
            TimeUpdateStrategy::FixedTimesteps(n) if *n == fast_forward_steps_per_update(step)
        ));
    }

    #[test]
    fn wall_paced_policies_keep_reading_the_wall_clock() {
        for policy in [TimePolicy::RealTime, TimePolicy::Scaled(10.0)] {
            let world = paced_world(400.0, policy);

            assert!(
                matches!(
                    world.resource::<TimeUpdateStrategy>(),
                    TimeUpdateStrategy::Automatic
                ),
                "{policy:?} must leave the clock on the wall"
            );
        }
    }

    #[test]
    fn fast_forward_clears_the_clamp_it_has_to_fit_inside() {
        // The clamp must exceed the synthetic delta the run actually feeds the
        // clock — step times batch — or `Time<Virtual>` drops the excess and the
        // run quietly simulates less than it was asked to.
        for frequency_hz in [1.0, 50.0, 400.0, 10_000.0] {
            let step = fixed_timestep(frequency_hz);
            let batch = fast_forward_steps_per_update(step);

            assert!(
                fast_forward_max_delta(step, batch) > step * batch,
                "{frequency_hz} Hz clamps its own synthetic delta"
            );
        }
    }

    #[test]
    fn a_slow_tick_rate_widens_the_clamp_when_fast_forwarding() {
        let step = fixed_timestep(SLOWER_THAN_DEFAULT_CLAMP_HZ);
        let default_clamp = Time::<Virtual>::default().max_delta();
        assert!(step > default_clamp, "the fixture must exceed the default");

        let widened =
            max_delta_after_pacing(SLOWER_THAN_DEFAULT_CLAMP_HZ, TimePolicy::FastAsPossible);

        assert!(widened > step);
    }

    #[test]
    fn wall_paced_policies_leave_the_clamp_alone() {
        // On a real clock the clamp is doing its actual job — bounding the
        // catch-up after a stalled frame — so neither policy may widen it.
        let default_clamp = Time::<Virtual>::default().max_delta();

        for policy in [TimePolicy::RealTime, TimePolicy::Scaled(10.0)] {
            assert_eq!(
                max_delta_after_pacing(SLOWER_THAN_DEFAULT_CLAMP_HZ, policy),
                default_clamp,
                "{policy:?} must not touch the clamp"
            );
        }
    }

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
                (|mut o: ResMut<SetOrder>| o.0.push("Validation"))
                    .in_set(SimulationSet::Validation),
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
                "Validation",
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
