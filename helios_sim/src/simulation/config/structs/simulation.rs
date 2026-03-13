// helios_sim/src/simulation/config/structs/simulation.rs
//
// New config sub-structs added for simulation profiles, metrics, and keybindings.

use serde::Deserialize;
use std::collections::HashMap;

/// Optional `[metrics]` table in the scenario TOML.
/// Controls where control-quality metrics are written on simulation exit.
#[derive(Debug, Deserialize, Default, Clone)]
#[serde(default)]
pub struct MetricsConfig {
    /// File path for CSV output. Omit to skip file output.
    pub output_path: Option<String>,
}

/// Optional `[debug.keybindings]` table.
/// Maps action IDs to key names, overriding built-in defaults.
///
/// Example:
/// ```toml
/// [debug.keybindings]
/// toggle_covariance  = "F2"
/// toggle_camera_feed = "F10"
/// ```
#[derive(Debug, Deserialize, Default, Clone)]
#[serde(default)]
pub struct KeybindingsConfig(pub HashMap<String, String>);
