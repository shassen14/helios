use serde::Deserialize;
use std::collections::HashMap;

use super::{EstimatorConfig, MapperConfig, PlannerConfig, SlamConfig, ControllerConfig};

#[derive(Debug, Deserialize, Default, Clone)]
#[serde(deny_unknown_fields)]
pub struct AutonomyStack {
    #[serde(default)]
    pub world_model: Option<WorldModelConfig>,

    #[serde(default)]
    pub planners: HashMap<String, PlannerConfig>,

    #[serde(default)]
    pub controllers: HashMap<String, ControllerConfig>,
}

/// Mutually exclusive ways to configure the world model.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum WorldModelConfig {
    CombinedSlam { slam: SlamConfig },
    Separate {
        estimator: Option<EstimatorConfig>,
        mapper: Option<MapperConfig>,
    },
}

impl Default for WorldModelConfig {
    fn default() -> Self {
        WorldModelConfig::Separate {
            estimator: None,
            mapper: None,
        }
    }
}
