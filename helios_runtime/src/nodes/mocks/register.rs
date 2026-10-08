//! Registers mock estimator factories.
//!
//! Mocks live in `helios_runtime` (not `helios_test`) because both
//! integration tests *and* dev iteration use them. A mock kind is selected
//! from TOML as a `[nodes.<name>]` entry, by `kind` like a real estimator.

use super::mock_oracle_estimator::MockOracleEstimatorNode;

use crate::assembly::AutonomyRegistry;
use crate::{BuildContext, FactoryOutput};

use serde::{Deserialize, Serialize};

/// The `kind` a profile writes to get a mock oracle estimator.
pub(crate) const MOCK_ORACLE_KIND: &str = "MockOracle";

/// The `[nodes.<name>]` section of a `MockOracle` entry. The node reads only
/// the body's oracle channels, so it takes no keys; the type exists so a
/// misspelled key is an error rather than silently ignored.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct MockOracleConfig {}

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node(MOCK_ORACLE_KIND, build)
        .expect("MockOracle is registered once, by the default registry")
}

/// Builds the node for one `[nodes]` entry. A body that doesn't publish the
/// oracle channels the node reads refuses it at pipeline build.
fn build(_config: MockOracleConfig, ctx: &BuildContext) -> Result<FactoryOutput, String> {
    Ok(FactoryOutput::new(Box::new(MockOracleEstimatorNode::new(
        ctx.node_name(),
        ctx.agent().clone(),
    ))))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::pipeline::node::PipelineNode;

    use helios_core::prelude::AgentId;

    use std::collections::HashSet;

    /// Builds a `[nodes]` entry named `name` from `section` through the
    /// default registry's node map, as the assembler does.
    fn build_entry(name: &str, section: &str) -> Result<Box<dyn PipelineNode>, String> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("test_agent"), name, &channels);
        AutonomyRegistry::default()
            .build_node(MOCK_ORACLE_KIND, section, &ctx)
            .map(|built| built.output.into_node())
            .map_err(|err| err.to_string())
    }

    // The node name is the table key, not the kind: two `MockOracle` entries
    // under distinct keys yield distinct node identities.
    #[test]
    fn a_nodes_entry_is_named_by_its_key_not_its_kind() {
        let truth = build_entry("truth", "").expect("an empty section builds");
        let shadow = build_entry("shadow", "").expect("an empty section builds");

        assert_eq!(truth.name(), "truth");
        assert_eq!(shadow.name(), "shadow");
    }

    #[test]
    fn a_nodes_entry_with_any_key_is_rejected() {
        assert!(build_entry("primary", "rate = 50.0").is_err());
    }
}
