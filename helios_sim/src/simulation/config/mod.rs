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
use crate::{
    cli::Cli,
    prelude::AppState,
    simulation::{config::structs::Simulation, core::app_state::AssetLoadSet},
};
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
                    info!("Successfully resolved agent: '{}'", agent_config.name());
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
    let mut final_config = ScenarioConfig {
        simulation: raw_config.simulation,
        world: raw_config.world,
        debug: raw_config.debug,
        metrics: raw_config.metrics,
        agents: resolved_agents,
    };

    apply_cli_overrides(&mut final_config.simulation, &cli);

    // 4. Insert the single, unified config as a resource.
    commands.insert_resource(final_config);
}

// Stamp command-line overrides onto the resolved simulation config, after the
// scenario file has been loaded. Precedence is CLI > file > default: a flag that
// is set wins over the file's value; a flag left unset leaves the file untouched.
// This is the single seam where launch-time flags reach the runtime config, so
// the resolved-config dump reflects exactly what ran.
fn apply_cli_overrides(sim: &mut Simulation, cli: &Cli) {
    // Only override when `--seed` was actually passed; otherwise the scenario
    // file's seed (or its absence) stands. Binding the value avoids re-reading
    // `cli.seed` through an `Option` after the assignment.
    if let Some(seed) = cli.seed {
        sim.seed = Some(seed);
        info!("seed from CLI override: {seed}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::PathBuf;

    // A `Cli` carrying only the seed under test; the path fields are irrelevant
    // to `apply_cli_overrides` and just need to be well-formed.
    fn cli_with_seed(seed: Option<u64>) -> Cli {
        Cli {
            scenario: PathBuf::from("unused.toml"),
            config_root: PathBuf::from("configs"),
            headless: true,
            seed,
        }
    }

    #[test]
    fn cli_seed_overrides_the_file_seed() {
        // CLI > file: an explicit `--seed` must win over the scenario value.
        let mut sim = Simulation {
            seed: Some(2024),
            ..Default::default()
        };
        apply_cli_overrides(&mut sim, &cli_with_seed(Some(99)));
        assert_eq!(sim.seed, Some(99));
    }

    #[test]
    fn absent_cli_seed_keeps_the_file_seed() {
        // No `--seed` passed: the scenario file's seed stands untouched.
        let mut sim = Simulation {
            seed: Some(2024),
            ..Default::default()
        };
        apply_cli_overrides(&mut sim, &cli_with_seed(None));
        assert_eq!(sim.seed, Some(2024));
    }

    #[test]
    fn absent_cli_seed_leaves_an_unset_file_seed_unset() {
        // No `--seed` and no file seed: stays `None` so the RNG later falls back
        // to OS entropy.
        let mut sim = Simulation {
            seed: None,
            ..Default::default()
        };
        apply_cli_overrides(&mut sim, &cli_with_seed(None));
        assert_eq!(sim.seed, None);
    }

    #[test]
    fn cli_seed_sets_a_previously_unset_file_seed() {
        // `--seed` on a scenario that left seed unset still takes effect.
        let mut sim = Simulation {
            seed: None,
            ..Default::default()
        };
        apply_cli_overrides(&mut sim, &cli_with_seed(Some(7)));
        assert_eq!(sim.seed, Some(7));
    }
}
