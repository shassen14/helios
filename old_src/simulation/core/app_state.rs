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

    /// Pass 8: Final validation checks.
    Validation,

    /// Pass 9: Remove all temporary request components.
    Cleanup,
}
