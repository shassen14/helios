// helios_sim/src/simulation/config/mod.rs

//! This module handles loading, resolving, and validating all simulation
//! configuration from disk, including the prefab catalog system.

mod catalog;
mod resolver;

pub mod structs;

use bevy::prelude::*;
use figment::{
    providers::{Format, Toml},
    value::Value,
    Figment,
};

// Re-export public types
use crate::{cli::Cli, prelude::AppState, simulation::core::app_state::AssetLoadSet};
use catalog::load_catalog_from_disk;
pub use catalog::PrefabCatalog;
pub use structs::{AgentConfig, RawScenarioConfig, ScenarioConfig};

pub struct ConfigPlugin;

impl Plugin for ConfigPlugin {
    fn build(&self, app: &mut App) {
        app
            // The resource for the raw, unresolved catalog data.
            .init_resource::<PrefabCatalog>()
            // The resource for the top-level scenario config.
            .init_resource::<ScenarioConfig>()
            // Add all the systems that run at startup to load and process config.
            .add_systems(
                OnEnter(AppState::AssetLoading),
                (load_catalog_from_disk, load_and_resolve_scenario)
                    .chain()
                    .in_set(AssetLoadSet::Config),
            );
    }
}

fn load_and_resolve_scenario(mut commands: Commands, cli: Res<Cli>, catalog: Res<PrefabCatalog>) {
    let scenario_path = &cli.scenario;
    info!("Loading scenario from: {:?}", scenario_path);

    // 1. Load the file into the temporary `RawScenarioConfig`.
    let raw_config: RawScenarioConfig =
        match Figment::new().merge(Toml::file(scenario_path)).extract() {
            Ok(s) => s,
            Err(e) => panic!("Failed to load or parse scenario file: {}", e),
        };

    // 2. Resolve the raw agent values into a Vec<AgentConfig>.
    let mut resolved_agents: Vec<AgentConfig> = Vec::new();
    for agent_value in &raw_config.agents {
        match resolver::resolve_agent_value(agent_value, &catalog) {
            Ok(resolved_value) => match Value::deserialize::<AgentConfig>(&resolved_value) {
                Ok(agent_config) => {
                    info!("Successfully resolved agent: '{}'", &agent_config.name);
                    resolved_agents.push(agent_config);
                }
                Err(e) => error!(
                    "Failed to deserialize resolved agent: {}. Value was: {:?}",
                    e, resolved_value
                ),
            },
            Err(e) => error!("Failed to resolve agent: {}", e),
        }
    }

    // 3. Assemble the final, complete `ScenarioConfig` resource.
    let final_config = ScenarioConfig {
        simulation: raw_config.simulation,
        world: raw_config.world,
        agents: resolved_agents,
    };

    // 4. Insert the single, unified config as a resource.
    commands.insert_resource(final_config);
}

fn transition_to_scene_building(mut next_state: ResMut<NextState<AppState>>) {
    info!("Configuration loading and resolution complete. Transitioning to SceneBuilding state.");
    next_state.set(AppState::SceneBuilding);
}
