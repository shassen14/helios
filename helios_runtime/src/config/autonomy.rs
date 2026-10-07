use super::{
    AllocatorConfig, ControllerConfig, EstimatorConfig, MapLayerConfig, PathFollowingConfig,
    ReferenceArbitrationConfig, SearchPlannerConfig, TeleopMapperConfig, TfBufferConfig,
};

use serde::Deserialize;
use std::collections::{BTreeMap, HashMap};

#[derive(Debug, Deserialize, Default, Clone)]
#[serde(deny_unknown_fields)]
pub struct AutonomyStack {
    /// Pipeline nodes, one `[nodes.<name>]` table each. The key is the node
    /// name; the table holds its `kind` plus that kind's own keys, left
    /// untyped here because only the kind's factory knows its shape. Sorted, so
    /// nodes are built, reported and dumped in the same order every run.
    #[serde(default)]
    pub nodes: BTreeMap<String, toml::Table>,

    /// Ego localization — named estimator instances.
    /// Key is the instance name (e.g. `"primary"`); value is the estimator config.
    /// Most agents have exactly one entry. Multiple entries are valid for research
    /// comparisons but each must publish to a distinct output channel.
    #[serde(default)]
    pub estimators: HashMap<String, EstimatorConfig>,

    /// World building — one entry per named map layer (e.g. `"local"`, `"global"`).
    /// The HashMap key becomes the channel qualifier: `MapData @ "<key>"`.
    #[serde(default)]
    pub map_layers: HashMap<String, MapLayerConfig>,

    #[serde(default)]
    pub search_planners: HashMap<String, SearchPlannerConfig>,

    #[serde(default)]
    pub path_following: Option<PathFollowingConfig>,

    #[serde(default)]
    pub controllers: HashMap<String, ControllerConfig>,

    #[serde(default)]
    pub allocators: HashMap<String, AllocatorConfig>,

    #[serde(default)]
    pub teleop: Option<TeleopMapperConfig>,

    /// Reference-arbitration tuning (teleop-vs-autonomy freshness). Defaults apply
    /// when the `[reference_arbitration]` section is omitted, so a stack with no
    /// teleop source never has to mention it.
    #[serde(default)]
    pub reference_arbitration: ReferenceArbitrationConfig,

    /// Sizing for the estimated transform buffer the `TfService` folds dual-
    /// published edges into. Defaults apply when the `[tf]` section is omitted.
    #[serde(default)]
    pub tf: TfBufferConfig,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `[nodes.<name>]` table loads with every key intact, `kind` included,
    /// and a stack without `[nodes]` loads with none.
    #[test]
    fn nodes_tables_load_untyped_and_default_to_empty() {
        let stack: AutonomyStack = toml::from_str(
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

        let empty: AutonomyStack = toml::from_str("").expect("empty stack parses");
        assert!(empty.nodes.is_empty());
    }
}
