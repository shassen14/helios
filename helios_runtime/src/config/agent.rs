use serde::Deserialize;

use super::AutonomyStack;

/// Portable agent identity and autonomy configuration.
///
/// Contains no simulation-specific fields (no vehicle, sensors, or physics).
/// Can be loaded by `helios_hw` directly from `configs/catalog/agent_profiles/`
/// without any `helios_sim` dependency.
#[derive(Debug, Deserialize, Clone)]
pub struct AgentBaseConfig {
    pub name: String,
    #[serde(default)]
    pub autonomy_stack: AutonomyStack,
}
