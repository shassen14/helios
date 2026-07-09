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

    /// The simulation is paused. Physics is stopped.
    Paused,

    /// Transitional state entered before application exit. I/O systems
    /// (metrics, data logger) flush to disk here via `OnEnter(Flushing)`,
    /// which Bevy guarantees to run before the next `Update` frame exits.
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

    /// Pass 2: Spawn all static world objects (signs, buildings, terrain features).
    /// Runs before agent processing so sensors can reference world geometry at setup time.
    ProcessWorldObjects,

    /// Pass 3: Process basic vehicle logic and add the DynamicsModel.
    ProcessVehicle,

    /// Pass 4: Process all sensor requests and create sensor child entities.
    ProcessSensors,

    /// Pass 5: Process "base" autonomy modules that have no other autonomy dependencies.
    /// (Estimators, Mappers, SLAM modules).
    ProcessBaseAutonomy,

    /// Pass 6: Process modules that depend on the base autonomy layer.
    /// (Planners, Trackers).
    ProcessDependentAutonomy,

    /// Pass 7: Process modules that depend on the planners.
    /// (Controllers).
    ProcessControllers,

    /// Pass 8: Attach all physical bodies (RigidBody, Collider).
    Physics,

    /// Pass 9: TODO:
    Finalize,

    /// Pass 10: Final validation checks.
    Validation,

    /// Pass 11: Remove all temporary request components.
    Cleanup,
}

// =========================================================================
// == Main Simulation Sets (The "Data Flow Graph") ==
// =========================================================================

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum SimulationSet {
    /// Prepare per-frame data before anything reads it: rebuild the TF tree from
    /// the current physics transforms. Runs first.
    Precomputation,

    /// Simulate raw physical sensors (IMU, GPS, LiDAR, …) and publish
    /// `SensorReading` batches to the pipeline bus.
    Sensors,

    /// The whole-brain tick: `run_pipeline_tick` fires every DAG node (estimator,
    /// mapper, planner, path-follower, controller) in topological order, each
    /// gated by its own `RateTimer`. This is the entire autonomy pipeline for
    /// every agent, not just estimation.
    BrainTick,

    /// Host → pipeline input glue: feeds the pipeline's mission-goal slot
    /// (`dispatch_configured_goals`, then `forward_goal_events`). Runs *after*
    /// `BrainTick`, so an injected goal takes effect on the next tick — the name
    /// describes data direction (into the pipeline), not temporal order.
    BrainInput,

    /// Pipeline → host output glue: copies `pipeline.read_control()` into
    /// `ControlOutputComponent` for the vehicle adapter to consume.
    BrainOutput,

    /// Convert control commands into physical forces (vehicle adapter → Avian3D
    /// `ExternalForce` / `ExternalTorque`).
    Actuation,

    /// Sync ground-truth state back from Avian3D after the physics step.
    StateSync,
}
