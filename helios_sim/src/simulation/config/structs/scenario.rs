use bevy::prelude::Resource;
use figment::value::Value;
use helios_runtime::config::{AgentBaseConfig, AutonomyStack};
use serde::Deserialize;
use std::collections::HashMap;

use super::{
    pose::Pose,
    sensors::SensorConfig,
    simulation::{KeybindingsConfig, MetricsConfig},
    terrain::{AtmosphereConfig, TerrainConfig},
    vehicle::Vehicle,
    world_object::WorldObjectPlacement,
};

/// The primary Bevy resource holding all configuration for a simulation run.
#[derive(Resource, Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct ScenarioConfig {
    #[serde(default)]
    pub simulation: Simulation,

    #[serde(default)]
    pub world: World,

    #[serde(default)]
    pub debug: DebugConfig,

    #[serde(default)]
    pub metrics: MetricsConfig,

    #[serde(default)]
    pub agents: Vec<AgentConfig>,
}

/// Optional `[debug]` table in the scenario TOML.
#[derive(Debug, Deserialize, Default, Clone)]
#[serde(default)]
pub struct DebugConfig {
    pub show_pose_gimbals: bool,
    pub show_covariance: bool,
    pub show_point_cloud: bool,
    pub show_velocity: bool,
    pub show_error_line: bool,
    pub show_path_trail: bool,
    pub show_occupancy_grid: bool,
    pub show_tf_frames: bool,
    pub show_planned_path: bool,
    pub show_legend: bool,
    /// Optional key override table, e.g. `toggle_covariance = "F2"`.
    #[serde(default)]
    pub keybindings: KeybindingsConfig,
}

/// Temporary helper for the initial file-loading step before agent prefab resolution.
#[derive(Deserialize)]
pub struct RawScenarioConfig {
    #[serde(default)]
    pub simulation: Simulation,
    #[serde(default)]
    pub world: World,
    #[serde(default)]
    pub debug: DebugConfig,
    #[serde(default)]
    pub metrics: MetricsConfig,
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
