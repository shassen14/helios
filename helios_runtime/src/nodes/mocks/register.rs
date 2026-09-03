//! Registers mock estimator factories.
//!
//! Mocks live in `helios_runtime` (not `helios_test`) because both
//! integration tests *and* dev iteration use them. A mock kind is selected
//! from TOML via the same `kind = "..."` mechanism as a real EKF.

use super::mock_oracle_estimator::MockOracleEstimatorNode;

use crate::config::EstimatorConfig;
use crate::pipeline::node::PipelineNode;
use crate::registry::{contexts::MockEstimatorBuildContext, AutonomyRegistry};

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_mock_estimator("MockOracle", build_mock_oracle);
}

fn build_mock_oracle(
    config: EstimatorConfig,
    ctx: MockEstimatorBuildContext,
) -> Result<Box<dyn PipelineNode>, String> {
    // The factory is keyed by kind string; the variant should match.
    // Mismatched variants are a registration error, not a user-facing one.
    let EstimatorConfig::MockOracle(_cfg) = config else {
        return Err("build_mock_oracle received non-MockOracle config".to_string());
    };
    Ok(Box::new(MockOracleEstimatorNode::new(
        ctx.instance_name,
        ctx.agent_handle,
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::config::MockOracleEstimatorConfig;

    use helios_core::data::primitives::FrameHandle;

    fn context(instance_name: &str) -> MockEstimatorBuildContext {
        MockEstimatorBuildContext {
            agent_handle: FrameHandle(0),
            instance_name: instance_name.to_string(),
        }
    }

    // The node name is the config-map key, not the kind: two `MockOracle`
    // estimators under distinct keys must yield distinct node identities.
    #[test]
    fn node_name_is_the_config_key_not_the_kind() {
        let config = EstimatorConfig::MockOracle(MockOracleEstimatorConfig {});
        let truth = build_mock_oracle(config.clone(), context("truth")).unwrap();
        let shadow = build_mock_oracle(config, context("shadow")).unwrap();

        assert_eq!(truth.name(), "truth");
        assert_eq!(shadow.name(), "shadow");
    }
}
