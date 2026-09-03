use super::{
    AllocatorConfig, ControllerConfig, EstimatorConfig, MapLayerConfig, PathFollowingConfig,
    ReferenceArbitrationConfig, SearchPlannerConfig, TeleopMapperConfig,
};

use serde::Deserialize;
use std::collections::HashMap;

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

    #[serde(default)]
    pub allocators: HashMap<String, AllocatorConfig>,

    #[serde(default)]
    pub teleop: Option<TeleopMapperConfig>,

    /// Reference-arbitration tuning (teleop-vs-autonomy freshness). Defaults apply
    /// when the `[reference_arbitration]` section is omitted, so a stack with no
    /// teleop source never has to mention it.
    #[serde(default)]
    pub reference_arbitration: ReferenceArbitrationConfig,
}
