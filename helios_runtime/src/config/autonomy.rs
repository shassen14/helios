use serde::Deserialize;
use std::collections::HashMap;

use super::{
    ControllerConfig, EstimatorConfig, MapLayerConfig, PathFollowingConfig, SearchPlannerConfig,
};

#[derive(Debug, Deserialize, Default, Clone)]
#[serde(deny_unknown_fields)]
pub struct AutonomyStack {
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
}
