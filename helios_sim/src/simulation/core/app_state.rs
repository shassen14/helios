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

    /// Pass 2: Process basic vehicle logic and add the DynamicsModel.
    ProcessVehicle,

    /// Pass 3: Process all sensor requests and create sensor child entities.
    ProcessSensors,

    /// Pass 4: Process "base" autonomy modules that have no other autonomy dependencies.
    /// (Estimators, Mappers, SLAM modules).
    ProcessBaseAutonomy,

    /// Pass 5: Process modules that depend on the base autonomy layer.
    /// (Planners, Trackers).
    ProcessDependentAutonomy,

    /// Pass 6: Process modules that depend on the planners.
    /// (Controllers).
    ProcessControllers,

    /// Pass 7: Attach all physical bodies (RigidBody, Collider).
    Physics,

    /// Pass 8: TODO:
    Finalize,

    /// Pass 9: Final validation checks.
    Validation,

    /// Pass 10: Remove all temporary request components.
    Cleanup,
}

// =========================================================================
// == Main Simulation Sets (The "Data Flow Graph") ==
// =========================================================================

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum SimulationSet {
    // --- Phase 1: Data Preparation ---
    /// Systems that prepare data for the rest of the frame. Runs first.
    /// Example: `tf_tree_builder_system`.
    Precomputation,

    // --- Phase 2: Perception (Can run in parallel) ---
    /// Systems that simulate raw physical sensors (IMU, GPS, LiDAR, Cameras).
    Sensors,
    /// Systems that process raw sensor data into detections (Object Detection, Segmentation).
    /// This can run in parallel with raw sensor simulation.
    Perception,

    // --- Phase 3: World Modeling (Depends on Perception) ---
    /// Systems that build geometric or semantic representations of the world.
    /// This includes SLAM, Mapping, and Tracking. This set runs *after* Perception.
    WorldModeling,

    // --- Phase 4: State & Decision Making (Depends on World Modeling) ---
    /// The core state estimator (EKF, UKF). It needs the latest sensor data and world model.
    Estimation,
    /// High-level decision making (Behavior Trees). Runs *after* the state is estimated.
    Behavior,

    // --- Phase 5: Motion (Depends on Decision Making) ---
    /// Path planning systems (A*, RRT*). Runs *after* the Behavior Tree sets a goal.
    Planning,
    /// Motion control systems (PID, MPC). Runs *after* a path is planned.
    Control,

    // --- Phase 6: Finalization ---
    /// Systems that convert control commands into physical forces. Runs last.
    Actuation,

    // --- Phase 7: Synchronization ---
    /// Systems that sync the ground truth state
    StateSync,

    Validation,
}
