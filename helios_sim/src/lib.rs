// helios_sim/src/lib.rs

use bevy::prelude::*;

// Import the plugins defined within the simulation crate.
use crate::simulation::core::simulation_setup::SimulationSetupPlugin;
use crate::simulation::plugins::debugging::gimbal::DebugGimbalPlugin;
use crate::simulation::plugins::sensors::gps::GpsPlugin;
// use crate::simulation::plugins::estimation::ekf::EkfPlugin;
use crate::simulation::plugins::sensors::imu::ImuPlugin;
use crate::simulation::plugins::vehicles::ackermann::AckermannCarPlugin;
use crate::simulation::plugins::world::spawner::WorldSpawnerPlugin;
use crate::simulation::plugins::world_model::WorldModelPlugin;

// This prelude is for convenience for other files WITHIN the helios_sim crate.
pub mod prelude;

// This module contains all the simulation-specific logic.
pub mod simulation;

/// The main plugin that brings together all the simulation parts.
/// Your `main.rs` will just add this one plugin to the Bevy App.
pub struct HeliosSimulationPlugin;

impl Plugin for HeliosSimulationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            // Core setup (spawns agents, sets up stages, etc.)
            SimulationSetupPlugin,
            // Spawns the world mesh, lighting, camera.
            WorldSpawnerPlugin,
            // Adds the Ackermann vehicle logic.
            AckermannCarPlugin,
            // Adds the IMU sensor logic.
            ImuPlugin,
            // Adds the GPS sensor logic.
            GpsPlugin,
            // Adds the EKF estimation logic.
            WorldModelPlugin,
            DebugGimbalPlugin,
            // When you add new plugins (Lidar, Planners), you will add them here.
        ));
    }
}
