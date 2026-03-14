use bevy::prelude::*;

use crate::simulation::profile::SimulationProfile;
use crate::simulation::profile_plugin::ProfiledSimulationPlugin;

// This prelude is for convenience for other files WITHIN the helios_sim crate.
pub mod prelude;

// This module contains all the simulation-specific logic.
pub mod cli;
pub mod simulation;

/// The main plugin that brings together all simulation subsystems.
///
/// Delegates to `ProfiledSimulationPlugin` with `SimulationProfile::FullPipeline`,
/// providing identical behavior to the pre-refactor monolithic plugin.
/// Use `ProfiledSimulationPlugin` directly when you need a non-default profile.
pub struct HeliosSimulationPlugin;

impl Plugin for HeliosSimulationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(ProfiledSimulationPlugin {
            profile: SimulationProfile::FullPipeline,
        });
    }
}
