use serde::Deserialize;

fn default_lookahead_distance_m() -> f64 {
    2.0
}
fn default_lookahead_time_s() -> f64 {
    1.0
}
fn default_goal_radius() -> f64 {
    3.0
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum PathFollowingStateSourceConfig {
    #[default]
    Estimated,
    GroundTruth,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum PathFollowingConfig {
    PurePursuit {
        max_speed_m_s: f64,
        min_speed_m_s: f64,
        #[serde(default = "default_lookahead_distance_m")]
        lookahead_distance_m: f64,
        #[serde(default = "default_lookahead_time_s")]
        lookahead_time_s: f64,
        #[serde(default = "default_goal_radius")]
        goal_radius: f64,
        #[serde(default)]
        state_source: PathFollowingStateSourceConfig,
    },
}

impl PathFollowingConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            PathFollowingConfig::PurePursuit { .. } => "PurePursuit",
        }
    }
    pub fn state_source(&self) -> PathFollowingStateSourceConfig {
        match self {
            PathFollowingConfig::PurePursuit { state_source, .. } => *state_source,
        }
    }
}
