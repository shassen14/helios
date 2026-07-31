// New config sub-structs added for simulation profiles, metrics, and keybindings.

use serde::Deserialize;

/// Optional `[metrics]` table in the scenario TOML.
/// Controls where control-quality metrics are written on simulation exit.
#[derive(Debug, Deserialize, Default, Clone)]
#[serde(default, deny_unknown_fields)]
pub struct MetricsConfig {
    /// File path for CSV output. Omit to skip file output.
    pub output_path: Option<String>,
}
