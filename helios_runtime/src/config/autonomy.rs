use serde::Deserialize;
use std::collections::HashMap;

use super::{
    ControllerConfig, EstimatorConfig, GeometricPlannerConfig, MapLayerConfig, PathFollowingConfig,
};

#[derive(Debug, Deserialize, Default, Clone)]
#[serde(deny_unknown_fields)]
pub struct AutonomyStack {
    /// Ego localization — state estimator (EKF, UKF, etc.).
    #[serde(default)]
    pub estimator: Option<EstimatorConfig>,

    /// World building — one entry per named map layer (e.g. "local", "global").
    /// The HashMap key becomes the channel name: `MapData @ "<key>"`.
    #[serde(default)]
    pub map_layers: HashMap<String, MapLayerConfig>,

    #[serde(default)]
    pub geometric_planners: HashMap<String, GeometricPlannerConfig>,

    #[serde(default)]
    pub path_following: Option<PathFollowingConfig>,

    #[serde(default)]
    pub controllers: HashMap<String, ControllerConfig>,
}
