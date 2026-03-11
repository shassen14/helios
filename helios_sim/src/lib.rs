use bevy::prelude::*;

use crate::simulation::core::simulation_setup::SimulationSetupPlugin;
use crate::simulation::core::TopicBusPlugin;
use crate::simulation::plugins::debugging::DebuggingPlugin;
use crate::simulation::plugins::foxglove::FoxgloveWebSocketPlugin;
use crate::simulation::plugins::sensors::gps::GpsPlugin;
use crate::simulation::plugins::sensors::imu::ImuPlugin;
use crate::simulation::plugins::sensors::magnetometer::MagnetometerPlugin;
use crate::simulation::plugins::sensors::raycasting::RaycastingSensorPlugin;
use crate::simulation::plugins::control::ControlPlugin;
use crate::simulation::plugins::vehicles::ackermann::AckermannCarPlugin;
use crate::simulation::plugins::world::AtmospherePlugin;
use crate::simulation::plugins::world::TerrainPlugin;
use crate::simulation::plugins::world::WorldObjectPlugin;
use crate::simulation::plugins::autonomy::AutonomyPlugin;
use crate::simulation::registry::plugin::AutonomyRegistryPlugin;

// This prelude is for convenience for other files WITHIN the helios_sim crate.
pub mod prelude;

// This module contains all the simulation-specific logic.
pub mod cli;
pub mod simulation;

/// The main plugin that brings together all the simulation parts.
pub struct HeliosSimulationPlugin;

impl Plugin for HeliosSimulationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            TopicBusPlugin,
            SimulationSetupPlugin,
            AutonomyRegistryPlugin,
            // World: terrain tiles (owns the AssetLoading→SceneBuilding gate).
            TerrainPlugin,
            // World: atmosphere — lighting, gravity, camera.
            AtmospherePlugin,
            // World: discrete semantic objects (signs, buildings, trees, etc.).
            WorldObjectPlugin,
            AckermannCarPlugin,
            ImuPlugin,
            GpsPlugin,
            MagnetometerPlugin,
            RaycastingSensorPlugin,
            AutonomyPlugin,
            ControlPlugin,
            DebuggingPlugin,
            FoxgloveWebSocketPlugin::default(),
        ));
    }
}