// src/simulation/core/app_state.rs

use bevy::{ecs::schedule::SystemSet, prelude::States};

/// Defines the major phases of the application's lifecycle.
#[derive(States, Debug, Clone, Eq, PartialEq, Hash, Default)]
pub enum AppState {
    /// The initial state. The app starts here.
    #[default]
    AssetLoading,

    /// All assets are loaded. We are now creating topics on the bus
    /// and building the scene by spawning entities from the config.
    SceneBuilding,

    /// The scene is built. The main simulation loop is now running.
    Running,

    /// Terminal state entered once a run is finished (target reached, timed out,
    /// or aborted). Report/finalize/exit work hangs off `OnEnter(Flushing)`, so
    /// it runs exactly once after the last tick and before the app exits. Nothing
    /// inside this crate drives the transition — a host or test harness sets it.
    Flushing,
}

/// System sets to control the order of execution during the AssetLoading state.
/// This ensures configuration is loaded before assets that depend on it are requested.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum AssetLoadSet {
    /// Stage 1: Load and resolve all TOML configuration files from disk.
    Config,
    /// Stage 2: Kick off the loading of assets (like GLTFs) that are specified in the config.
    Kickoff,
    /// Stage 3: Systems that run on `Update` to check for asset load completion.
    Check,
}

/// System sets to control the order of execution during the SceneBuilding state.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum SceneBuildSet {
    /// Pass 1: Create agent shells and attach the main request component.
    CreateRequests,

    /// Pass 2: Spawn the world layout's objects, the ground included.
    /// Runs before agent processing so sensors can reference world geometry at setup time.
    ProcessWorldObjects,

    /// Pass 3: Process basic vehicle logic and add the DynamicsModel.
    ProcessVehicle,

    /// Pass 4: Process all sensor requests and create sensor child entities.
    /// Attaches the sensor's forward model (from `helios_core::sensors`, never
    /// a `MeasurementModel`), `TrackedFrame`, and `SensorPublishChannel`.
    /// Runs after `ProcessWorldObjects`, which settles the environmental
    /// constants (gravity, the magnetic field) that sensors capture at spawn.
    ProcessSensors,

    /// Pass 5: Build each agent's `AutonomyPipeline` and attach
    /// `AutonomyPipelineComponent`. Runs after `ProcessSensors` because pipeline
    /// assembly collects the sensors' `SensorPublishChannel` descriptions into
    /// the body.
    SpawnPipeline,

    /// Pass 6: Bind host-side ECS state to the freshly-spawned pipeline — the
    /// `ActuatorCommandComponent` the vehicle plugin drains, and the watch set
    /// the observation sinks asked for. Both only require
    /// `AutonomyPipelineComponent` to exist (from `SpawnPipeline`) and are
    /// independent of each other, so they share this one pass.
    BindPipeline,

    /// Pass 7: Attach all physical bodies (RigidBody, Collider).
    Physics,

    /// Pass 8: Snapshot the static TF tree now that the full entity hierarchy
    /// exists, so pose lookups have a complete frame graph before `Running`.
    Finalize,

    /// Pass 9: Remove all temporary request components, then transition to `Running`.
    Cleanup,
}

// =========================================================================
// == Main Simulation Sets (The "Data Flow Graph") ==
// =========================================================================

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum SimulationSet {
    /// Reserved: per-frame data prepared before anything reads it. Nothing runs
    /// here yet. Runs first.
    Precomputation,

    /// Apply the command the brain decided last step as physical forces
    /// (vehicle plugin → Avian3D `ConstantForce` / `ConstantTorque`). Physics
    /// runs right after, carrying the world from the last step's instant to
    /// this one's.
    Actuation,

    /// Sync ground truth from Avian3D after the physics step and transform
    /// propagation, setting `GroundTruthState::at` to the step's instant, and
    /// publish the truth channels stamped with it.
    StateSync,

    /// Simulate raw physical sensors (IMU, GPS, LiDAR, …) on the post-step
    /// world and publish `SensorReading` batches to the pipeline bus, stamped
    /// with their agent's `GroundTruthState::at`.
    Sensors,

    /// The whole-brain tick: `run_pipeline_tick` fires every DAG node (estimator,
    /// mapper, planner, path-follower, controller) in topological order, each
    /// gated by its own `RateTimer`, at `now = GroundTruthState::at`. This is
    /// the entire autonomy pipeline for every agent, not just estimation.
    BrainTick,

    /// Host → pipeline input glue: feeds the pipeline's mission-goal slot
    /// (`dispatch_configured_goals`, then `forward_goal_events`). Runs *after*
    /// `BrainTick`, so an injected goal takes effect on the next tick — the name
    /// describes data direction (into the pipeline), not temporal order.
    BrainInput,

    /// Pipeline → host output glue: copies `pipeline.read_actuators()` into
    /// `ActuatorCommandComponent` for the next step's `Actuation` to apply.
    BrainOutput,

    /// Hand the tick's observation batch to every registered sink. Before
    /// `Validation`, so a per-tick check reads sinks that already hold the
    /// tick.
    Observation,

    /// The step's last slot: runs once ground truth, the brain's tick and the
    /// oracle channels all describe the step's instant. External harnesses, data logging, and
    /// per-tick checks read the buses here. Empty within this crate itself —
    /// it exists so downstream consumers have a set that provably runs after
    /// everything else in the tick.
    Validation,
}
