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
    cli::Cli, config::structs::Simulation, core::app_state::AssetLoadSet, prelude::AppState,
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

/// Takes one raw agent value all the way to a typed [`AgentConfig`], resolving
/// its `from` references and then checking the result against the schema.
///
/// The two failures are labelled distinctly because they send the reader to
/// different files. A resolution failure means a `from` reference did not land
/// — the referenced prefab is missing or misspelled. A deserialization failure
/// means the reference resolved fine but the fields it produced do not match
/// what `AgentConfig` expects, and serde's message names the offending field
/// and the valid alternatives.
fn resolve_agent(agent_value: &Value, catalog: &PrefabCatalog) -> Result<AgentConfig, String> {
    let resolved_value = resolver::resolve_agent_value(agent_value, catalog)
        .map_err(|e| format!("failed to resolve: {}", e))?;

    Value::deserialize::<AgentConfig>(&resolved_value)
        .map_err(|e| format!("failed to deserialize: {}", e))
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
    let mut failures: Vec<String> = Vec::new();

    for (index, agent_value) in raw_config.agents.iter().enumerate() {
        match resolve_agent(agent_value, &catalog) {
            Ok(agent_config) => {
                info!("Successfully resolved agent: '{}'", agent_config.name());
                resolved_agents.push(agent_config);
            }
            // Collected rather than reported, so a scenario with several broken
            // agents lists all of them in one run instead of one per edit.
            // The index identifies the agent because a failed agent has no
            // parsed name to report — that is the failure.
            Err(e) => failures.push(format!("agents[{}]: {}", index, e)),
        }
    }

    // A partially-built scene has no valid interpretation: the scenario states
    // what to simulate, and quietly simulating a subset answers a question
    // nobody asked. Left non-fatal, a config typo becomes a green run over an
    // empty world.
    if !failures.is_empty() {
        panic!(
            "{} of {} agents failed to load:\n{}",
            failures.len(),
            raw_config.agents.len(),
            failures.join("\n")
        );
    }

    // 3. Assemble the final, complete `ScenarioConfig` resource.
    let mut final_config = ScenarioConfig {
        simulation: raw_config.simulation,
        world: raw_config.world,
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

    use std::{collections::HashMap, path::PathBuf};

    // The smallest agent that satisfies `AgentConfig`: every field without a
    // serde default, and nothing else. Written as TOML rather than built as a
    // `Value` tree so the tests fail the same way a real scenario would — via
    // the same parse and the same deserialize.
    const MINIMAL_AGENT: &str = r#"
        name = "test_car"

        [starting_pose]
        [goal_pose]

        [vehicle]
        kind = "Ackermann"
        wheelbase = 2.7
        max_steering_angle = 30.0
        max_steering_rate = 90.0

        [sensors.gps]
        kind = "Gps"
        rate = 10.0
        channel = "gps/fix"
    "#;

    // Parses an agent's TOML into the raw `Value` the scenario loader hands to
    // `resolve_agent`.
    fn agent_value(toml: &str) -> Value {
        Figment::new()
            .merge(Toml::string(toml))
            .extract()
            .expect("test agent TOML should parse")
    }

    // These agents carry no `from` references, so resolution has nothing to look
    // up and an empty catalog is sufficient. A test needing a reference to
    // resolve would seed this map instead.
    fn empty_catalog() -> PrefabCatalog {
        PrefabCatalog(HashMap::new())
    }

    #[test]
    fn a_well_formed_agent_resolves() {
        // Anchors the other tests: without this, a `resolve_agent` that rejected
        // everything would satisfy both failure tests below.
        let agent = resolve_agent(&agent_value(MINIMAL_AGENT), &empty_catalog())
            .expect("the minimal agent should resolve");

        assert_eq!(agent.name(), "test_car");
    }

    #[test]
    fn an_unknown_sensor_field_is_rejected() {
        // The regression guard for a config typo silently dropping an agent. A
        // misspelled `noise_stddev` is the real instance that reached a green
        // run: `deny_unknown_fields` caught it, and the error was discarded.
        let typo =
            MINIMAL_AGENT.replace("rate = 10.0", "rate = 10.0\nnoise_stdev = [0.0, 0.0, 0.0]");

        let error = resolve_agent(&agent_value(&typo), &empty_catalog())
            .expect_err("an unknown field should be rejected");

        // Asserting on the field name, not the whole message: serde owns the
        // phrasing, but the offending field is what sends the reader to the
        // right line, and a message lacking it is useless however it is worded.
        assert!(
            error.contains("noise_stdev"),
            "error should name the offending field, got: {error}"
        );
    }

    #[test]
    fn an_unresolvable_reference_is_rejected() {
        // The other failure path. Worth its own test because the two are
        // labelled differently on purpose — this one points at a missing
        // catalog entry, not at a schema mismatch — and an implementation that
        // collapsed them would still pass the test above.
        let dangling = MINIMAL_AGENT.replace(
            r#"kind = "Gps""#,
            r#"from = "entities.sensors.does_not_exist""#,
        );

        let error = resolve_agent(&agent_value(&dangling), &empty_catalog())
            .expect_err("a dangling reference should be rejected");

        assert!(
            error.starts_with("failed to resolve"),
            "a missing prefab should fail resolution, not deserialization, got: {error}"
        );
    }

    // A `Cli` carrying only the seed under test; the path fields are irrelevant
    // to `apply_cli_overrides` and just need to be well-formed.
    fn cli_with_seed(seed: Option<u64>) -> Cli {
        Cli {
            scenario: PathBuf::from("unused.toml"),
            config_root: PathBuf::from("configs"),
            headless: true,
            speed: None,
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
