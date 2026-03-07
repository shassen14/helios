use bevy::prelude::Resource;
use figment::value::Value;
use serde::Deserialize;
use std::{collections::HashMap, path::PathBuf};

use super::{autonomy::AutonomyStack, pose::Pose, sensors::SensorConfig, vehicle::Vehicle};

/// The primary Bevy resource holding all configuration for a simulation run.
/// This is the root of the data parsed from a `scenario.toml` file.
#[derive(Resource, Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct ScenarioConfig {
    #[serde(default)]
    pub simulation: Simulation,

    #[serde(default)]
    pub world: World,

    #[serde(default)]
    pub debug: DebugConfig,

    /// `[[agents]]` in TOML becomes a Vec of resolved `AgentConfig` structs.
    #[serde(default)]
    pub agents: Vec<AgentConfig>,
}

/// Optional `[debug]` table in the scenario TOML.
/// Every field defaults to `false`, so the whole section can be omitted.
#[derive(Debug, Deserialize, Default, Clone)]
#[serde(deny_unknown_fields)]
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
    pub show_legend: bool,
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
}

impl Default for Simulation {
    fn default() -> Self {
        Self {
            seed: None,
            duration_seconds: 60.0,
            frequency_hz: default_frequency_hz(),
            log_topics: Vec::new(),
        }
    }
}

fn default_frequency_hz() -> f64 {
    400.0
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct World {
    pub map_file: PathBuf,
    pub gravity: [f32; 3],
}

impl Default for World {
    fn default() -> Self {
        Self {
            map_file: "assets/maps/default.gltf".into(),
            gravity: [0.0, -9.81, 0.0],
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct AgentConfig {
    pub name: String,
    pub starting_pose: Pose,
    pub goal_pose: Pose,
    pub vehicle: Vehicle,
    #[serde(default)]
    pub sensors: HashMap<String, SensorConfig>,
    #[serde(default)]
    pub autonomy_stack: AutonomyStack,
}
