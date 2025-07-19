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
use crate::prelude::AppState;
use catalog::load_catalog_from_disk;
pub use catalog::PrefabCatalog;
pub use structs::{AgentConfig, ScenarioConfig};

pub struct ConfigPlugin;

impl Plugin for ConfigPlugin {
    fn build(&self, app: &mut App) {
        app
            // A resource to hold the final, resolved agent configurations.
            .init_resource::<ResolvedAgents>()
            // The resource for the raw, unresolved catalog data.
            .init_resource::<PrefabCatalog>()
            // The resource for the top-level scenario config.
            .init_resource::<ScenarioConfig>()
            // Add all the systems that run at startup to load and process config.
            .add_systems(
                OnEnter(AppState::AssetLoading),
                (
                    load_catalog_from_disk,
                    load_and_resolve_scenario,
                    transition_to_scene_building,
                )
                    .chain(),
            );
    }
}

#[derive(Resource, Default, Debug)]
pub struct ResolvedAgents(pub Vec<AgentConfig>);

fn load_and_resolve_scenario(
    mut scenario_config: ResMut<ScenarioConfig>, // <-- We now mutate the main config resource
    catalog: Res<PrefabCatalog>,
    mut resolved_agents: ResMut<ResolvedAgents>,
) {
    // FIX 2: Use a hardcoded path for now instead of a CLI resource.
    let scenario_path = "assets/scenarios/simple_car_scenario.toml";
    info!("Loading scenario from: {}", scenario_path);

    // Load the entire scenario file into our main ScenarioConfig resource.
    let loaded_config: ScenarioConfig =
        match Figment::new().merge(Toml::file(scenario_path)).extract() {
            Ok(s) => s,
            Err(e) => {
                panic!(
                    "Failed to load or parse scenario file at {}: {}",
                    scenario_path, e
                );
            }
        };

    // FIX 1: Instead of inserting separate resources, just populate the fields
    // of the main ScenarioConfig resource.
    scenario_config.simulation = loaded_config.simulation;
    scenario_config.world = loaded_config.world;

    // --- Resolve each agent against the catalog ---
    for agent_value in &loaded_config.agents {
        match resolver::resolve_agent_value(agent_value, &catalog) {
            Ok(resolved_value) => {
                // FIX 3: Use `Value::deserialize` to convert the resolved Value to AgentConfig.
                match Value::deserialize(&resolved_value) {
                    Ok(agent_config) => {
                        let agent_config: AgentConfig = agent_config; // Ensure type annotation
                        info!(
                            "Successfully resolved and deserialized agent: '{}'",
                            &agent_config.name
                        );
                        resolved_agents.0.push(agent_config);
                    }
                    Err(e) => {
                        error!("Failed to deserialize resolved agent config: {}. Value was: {:?}. Skipping agent.", e, resolved_value);
                    }
                }
            }
            Err(e) => {
                error!("Failed to resolve agent config: {}. Skipping agent.", e);
            }
        }
    }
}

fn transition_to_scene_building(mut next_state: ResMut<NextState<AppState>>) {
    info!("Configuration loading and resolution complete. Transitioning to SceneBuilding state.");
    next_state.set(AppState::SceneBuilding);
}
