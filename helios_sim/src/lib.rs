// helios_sim/src/lib.rs

use bevy::prelude::*;

// Import the plugins defined within the simulation crate.
use crate::simulation::core::simulation_setup::SimulationSetupPlugin;
use crate::simulation::core::TopicBusPlugin;
use crate::simulation::plugins::debugging::DebuggingPlugin;
use crate::simulation::plugins::foxglove::FoxgloveWebSocketPlugin;
use crate::simulation::plugins::sensors::gps::GpsPlugin;
use crate::simulation::plugins::sensors::imu::ImuPlugin;
use crate::simulation::plugins::sensors::magnetometer::MagnetometerPlugin;
use crate::simulation::plugins::sensors::raycasting::RaycastingSensorPlugin;
use crate::simulation::plugins::vehicles::ackermann::AckermannCarPlugin;
use crate::simulation::plugins::world::spawner::WorldSpawnerPlugin;
use crate::simulation::plugins::world_model::WorldModelPlugin;
use crate::simulation::registry::plugin::AutonomyRegistryPlugin;

// This prelude is for convenience for other files WITHIN the helios_sim crate.
pub mod prelude;

// This module contains all the simulation-specific logic.
pub mod cli;
pub mod simulation;

/// The main plugin that brings together all the simulation parts.
/// Your `main.rs` will just add this one plugin to the Bevy App.
pub struct HeliosSimulationPlugin;

impl Plugin for HeliosSimulationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            // Registers TopicBus resource — must be first so all sensors can publish to it.
            TopicBusPlugin,
            // Core setup (spawns agents, sets up stages, etc.)
            SimulationSetupPlugin,
            // Algorithm factories — must come before WorldModelPlugin so the registry
            // is fully populated before any spawning system runs.
            AutonomyRegistryPlugin,
            // Spawns the world mesh, lighting, camera.
            WorldSpawnerPlugin,
            // Adds the Ackermann vehicle logic.
            AckermannCarPlugin,
            // Add Sensors
            ImuPlugin,
            GpsPlugin,
            MagnetometerPlugin,
            RaycastingSensorPlugin,
            // Add World Modeling / estimator and modeling
            WorldModelPlugin,
            DebuggingPlugin,
            // Foxglove WebSocket bridge — must be last so all topics are registered first.
            FoxgloveWebSocketPlugin::default(),
            // When you add new plugins (Lidar, Planners), you will add them here.
        ));
    }
}
