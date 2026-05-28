//! Registers mock estimator factories.
//!
//! Mocks live in `helios_runtime` (not `helios_test`) because both
//! integration tests *and* dev iteration use them. A mock kind is selected
//! from TOML via the same `kind = "..."` mechanism as a real EKF.

use crate::config::EstimatorConfig;
use crate::pipeline::node::PipelineNode;
use crate::pipeline::nodes::mock_oracle_estimator::MockOracleEstimatorNode;

use super::{contexts::MockEstimatorBuildContext, AutonomyRegistry};

pub fn register(registry: &mut AutonomyRegistry) {
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
        "mock_oracle_estimator",
        ctx.agent_handle,
    )))
}
