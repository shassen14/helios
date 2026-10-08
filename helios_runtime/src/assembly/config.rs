//! [`AutonomyStackConfig`], the root of an agent's autonomy TOML, and
//! [`AgentBaseConfig`], the portable agent profile that carries one.
//!
//! The stack has one field per section. Each section's type lives in the
//! `config.rs` of the module that reads it: a seam's in its seam folder, the
//! `[tf]` sizing in `tf_service`, a `[nodes]` kind's in its node folder.

use super::seams::actuators::ActuatorSeamConfig;
use super::seams::command::CommandFoldConfig;
use super::seams::estimate::EstimateSeamConfig;
use super::seams::reference::ReferenceSeamConfig;

use crate::tf_service::TfBufferConfig;

use serde::Deserialize;
use std::collections::BTreeMap;

/// An agent's autonomy stack: the `[nodes]` it runs and the seam sections that
/// join them. Zero Bevy or simulation types, so sim and hardware load it alike.
#[derive(Debug, Deserialize, Default, Clone)]
#[serde(deny_unknown_fields)]
pub struct AutonomyStackConfig {
    /// Pipeline nodes, one `[nodes.<name>]` table each. The key is the node
    /// name; the table holds its `kind` plus that kind's own keys, left
    /// untyped here because only the kind's factory knows its shape. Sorted, so
    /// nodes are built, reported and dumped in the same order every run.
    #[serde(default)]
    pub nodes: BTreeMap<String, toml::Table>,

    /// The estimate seam: which estimator's state the rest of the stack reads
    /// and the TF edge comes from. Omitted when the stack has no estimator.
    #[serde(default)]
    pub estimate: Option<EstimateSeamConfig>,

    /// The guidance reference seam: which `[nodes]` entries feed the reference
    /// the controllers track. Omitted when nothing in the graph produces one.
    #[serde(default)]
    pub reference: Option<ReferenceSeamConfig>,

    /// The command seam: one `[command.<fold>]` table per fold, keyed by the
    /// fold's name, each summing the commands of the `[nodes]` entries it
    /// lists. Sorted, so folds are built and reported in the same order
    /// every run. Empty when nothing is commanded.
    #[serde(default)]
    pub command: BTreeMap<String, CommandFoldConfig>,

    /// The actuator seam: which `[nodes]` entries' partial actuator commands
    /// are merged into the one the body applies. Omitted when nothing drives
    /// the body.
    #[serde(default)]
    pub actuators: Option<ActuatorSeamConfig>,

    /// Sizing for the estimated transform buffer the `TfService` folds dual-
    /// published edges into. Defaults apply when the `[tf]` section is omitted.
    #[serde(default)]
    pub tf: TfBufferConfig,
}

/// Portable agent identity and autonomy configuration.
///
/// Contains no simulation-specific fields (no vehicle, sensors, or physics).
/// Can be loaded by `helios_hw` directly from `configs/runtime/profiles/agent_profiles/`
/// without any `helios_sim` dependency.
#[derive(Debug, Deserialize, Clone)]
pub struct AgentBaseConfig {
    pub name: String,
    #[serde(default)]
    pub autonomy_stack: AutonomyStackConfig,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `[nodes.<name>]` table loads with every key intact, `kind` included,
    /// and a stack without `[nodes]` loads with none.
    #[test]
    fn nodes_tables_load_untyped_and_default_to_empty() {
        let stack: AutonomyStackConfig = toml::from_str(
            r#"
            [nodes.front_deproject]
            kind = "Deproject"
            input = "front_lidar"

            [nodes.accumulate]
            kind = "Accumulate"
            "#,
        )
        .expect("stack with nodes parses");
        assert_eq!(
            stack.nodes.keys().map(String::as_str).collect::<Vec<_>>(),
            ["accumulate", "front_deproject"]
        );
        let front = &stack.nodes["front_deproject"];
        assert_eq!(front["kind"].as_str(), Some("Deproject"));
        assert_eq!(front["input"].as_str(), Some("front_lidar"));

        let empty: AutonomyStackConfig = toml::from_str("").expect("empty stack parses");
        assert!(empty.nodes.is_empty());
    }
}
