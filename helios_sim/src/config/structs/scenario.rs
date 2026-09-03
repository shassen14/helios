use bevy::prelude::Resource;
use figment::value::Value;
use helios_runtime::config::{AgentBaseConfig, AutonomyStack};
use serde::Deserialize;
use std::collections::HashMap;

use crate::config::structs::CameraVantage;

use super::{
    pose::Pose,
    sensors::SensorConfig,
    simulation::MetricsConfig,
    terrain::{AtmosphereConfig, MagneticFieldConfig, TerrainConfig},
    vehicle::Vehicle,
    world_object::WorldObjectPlacement,
};

/// The primary Bevy resource holding all configuration for a simulation run.
///
/// Everything except `agents` lives in [`ScenarioCommon`], flattened in so the
/// TOML shape is unchanged (`[simulation]`, `[world]`, … stay top-level) while
/// the loader moves the whole shared block across in one step. `agents` is the
/// sole field that differs from [`RawScenarioConfig`]: here it is typed, there
/// it is still unresolved. (`deny_unknown_fields` cannot combine with
/// `flatten`; unknown-key rejection lives in the nested `[simulation]`/`[world]`
/// structs, which is where a real typo lands anyway.)
#[derive(Resource, Debug, Deserialize, Default)]
pub struct ScenarioConfig {
    #[serde(flatten)]
    pub common: ScenarioCommon,

    #[serde(default)]
    pub agents: Vec<AgentConfig>,
}

/// Every scenario field that is identical before and after agent resolution —
/// all of it but the agents. Both [`ScenarioConfig`] and [`RawScenarioConfig`]
/// embed this with `#[serde(flatten)]`, so a new top-level block is declared
/// here once and both structs plus the loader's assembly step pick it up with
/// no further edits.
#[derive(Debug, Deserialize, Default)]
pub struct ScenarioCommon {
    #[serde(default)]
    pub simulation: Simulation,

    #[serde(default)]
    pub world: World,

    #[serde(default)]
    pub metrics: MetricsConfig,

    #[serde(default)]
    pub camera: CameraVantage,
}

/// Temporary helper for the initial file-loading step before agent prefab
/// resolution: identical to [`ScenarioConfig`] but for `agents`, which are
/// still opaque `figment::Value`s awaiting `from`-reference resolution.
#[derive(Deserialize)]
pub struct RawScenarioConfig {
    #[serde(flatten)]
    pub common: ScenarioCommon,

    #[serde(default)]
    pub agents: Vec<Value>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Simulation {
    pub seed: Option<u64>,
    pub duration_seconds: f32,
    #[serde(default = "default_frequency_hz")]
    pub frequency_hz: f64,
    #[serde(default)]
    pub log_topics: Vec<String>,
    /// Optional profile name read from TOML; `--profile` CLI flag overrides this.
    pub profile: Option<String>,
    /// Path to a fixture TOML containing a baked path (used by MockPathInjectorPlugin).
    pub mock_path: Option<String>,
    /// Path to a fixture TOML containing a baked map (used by MockMapInjectorPlugin).
    pub mock_map: Option<String>,
}

impl Default for Simulation {
    fn default() -> Self {
        Self {
            seed: None,
            duration_seconds: 60.0,
            frequency_hz: default_frequency_hz(),
            log_topics: Vec::new(),
            profile: None,
            mock_path: None,
            mock_map: None,
        }
    }
}

fn default_frequency_hz() -> f64 {
    400.0
}

/// World-level configuration: terrain tiles, atmosphere, and placed objects.
#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct World {
    /// One or more terrain tiles that compose the physical ground.
    /// Declared with `[[world.terrains]]` in TOML.
    #[serde(default)]
    pub terrains: Vec<TerrainConfig>,

    /// Lighting, gravity, and atmospheric parameters.
    /// Declared as `[world.atmosphere]` in TOML.
    #[serde(default)]
    pub atmosphere: AtmosphereConfig,

    /// The geomagnetic reference field every magnetometer measures.
    /// Declared as `[world.magnetic_field]` in TOML.
    #[serde(default)]
    pub magnetic_field: MagneticFieldConfig,

    /// Static world objects (signs, buildings, trees, etc.).
    /// Declared with `[[world.objects]]` in TOML.
    #[serde(default)]
    pub objects: Vec<WorldObjectPlacement>,
}

/// Full agent configuration for helios_sim.
#[derive(Debug, Deserialize, Clone)]
pub struct AgentConfig {
    #[serde(flatten)]
    pub base: AgentBaseConfig,
    pub starting_pose: Pose,
    pub goal_pose: Pose,
    pub vehicle: Vehicle,
    #[serde(default)]
    pub sensors: HashMap<String, SensorConfig>,
}

impl AgentConfig {
    pub fn name(&self) -> &str {
        &self.base.name
    }

    pub fn autonomy_stack(&self) -> &AutonomyStack {
        &self.base.autonomy_stack
    }
}
