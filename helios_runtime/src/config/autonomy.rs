use super::{
    AllocatorConfig, ControllerConfig, EstimatorConfig, MapLayerConfig, PathFollowingConfig,
    PreprocessingConfig, ReferenceArbitrationConfig, SearchPlannerConfig, TeleopMapperConfig,
    TfBufferConfig,
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

    /// Measurement conditioning — named nodes that turn one channel into another
    /// before any algorithm reads it (e.g. a range field flattened to a point
    /// cloud). The key is the node name; each entry names its own input and
    /// output channels.
    #[serde(default)]
    pub preprocessing: HashMap<String, PreprocessingConfig>,

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
