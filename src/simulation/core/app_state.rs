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
    /// Pass 1: Spawns logical entities and attaches spawn requests.
    Logic,
    /// Pass 2: Systems that fulfill spawn requests by attaching physical components.
    Physics,
    /// Pass 3: Systems that validate the fully-spawned agent pipelines.
    Validation,
}
