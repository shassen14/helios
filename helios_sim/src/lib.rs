//! Bevy + Avian3D simulation host for the Helios autonomy framework.
//!
//! Wraps `helios_runtime` and `helios_core` in an ECS application. Provides physics,
//! sensor simulation, config loading (TOML), visualization (Bevy gizmos + Foxglove),
//! and the `AutonomyRegistry` that maps config strings to concrete algorithm factories.
//! Entry point: [`HeliosSimulationPlugin`], which adds every simulation
//! sub-plugin; behavior is selected by scenario/agent config, not by profile.

use std::path::PathBuf;

use bevy::prelude::*;

use crate::simulation::core::simulation_setup::SimulationSetupPlugin;
use crate::simulation::plugins::autonomy::EstimationPlugin;
use crate::simulation::plugins::control::ControlPlugin;
use crate::simulation::plugins::debugging::DebuggingPlugin;
use crate::simulation::plugins::planning::PlanningPlugin;
use crate::simulation::plugins::sensors::HeliosSensorsPlugin;
use crate::simulation::plugins::vehicles::HeliosVehiclesPlugin;
use crate::simulation::plugins::world::HeliosWorldPlugin;
use crate::simulation::registry::plugin::AutonomyRegistryPlugin;

// This prelude is for convenience for other files WITHIN the helios_sim crate.
pub mod prelude;

// This module contains all the simulation-specific logic.
pub mod cli;
pub mod host;
pub mod simulation;

/// The main plugin that brings together all simulation subsystems.
///
/// Adds every sub-plugin unconditionally. What actually *runs* for a given agent
/// is decided by the nodes its `autonomy_stack` config declares in the pipeline,
/// not by which plugins are present. The list order is irrelevant: system
/// execution order comes from the `SimulationSet` graph configured once in
/// `SimulationSetupPlugin`, not from plugin registration order.
pub struct HeliosSimulationPlugin;

impl Plugin for HeliosSimulationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            SimulationSetupPlugin,
            AutonomyRegistryPlugin,
            HeliosWorldPlugin,
            HeliosVehiclesPlugin,
            HeliosSensorsPlugin,
            EstimationPlugin,
            PlanningPlugin,
            ControlPlugin,
            DebuggingPlugin,
        ));
    }
}

/// Absolute path to this crate's `assets/` directory, the canonical home of
/// Helios sim assets (terrain/object GLBs).
///
/// Built from `CARGO_MANIFEST_DIR`, which the compiler expands to *this*
/// crate's manifest dir regardless of which binary calls it — so a bin in
/// another crate (e.g. `helios_test_sim`) still resolves assets here without a
/// `BEVY_ASSET_ROOT` override. Drivers feed this to `AssetPlugin.file_path`;
/// an absolute `file_path` overrides Bevy's base-path search, making asset
/// loading independent of the current working directory.
///
/// Caveat: this is the *source-tree* location baked in at compile time. A
/// binary deployed away from the repo (future `helios_hw` / packaging) won't
/// find assets here and must set `BEVY_ASSET_ROOT` to their real install path.
pub fn asset_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets")
}
