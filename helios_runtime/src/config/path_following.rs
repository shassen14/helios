use serde::Deserialize;

fn default_lookahead_distance_m() -> f64 {
    2.0
}

fn default_goal_radius() -> f64 {
    3.0
}

fn default_max_lateral_acceleration() -> f64 {
    2.0
}

fn default_kp() -> f64 {
    2.0
}

fn default_ki() -> f64 {
    0.01
}

fn default_kd() -> f64 {
    0.0
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
    SteeringPid {
        cruise_speed: f64,
        #[serde(default = "default_kp")]
        kp: f64,
        #[serde(default = "default_ki")]
        ki: f64,
        #[serde(default = "default_kd")]
        kd: f64,
        #[serde(default = "default_goal_radius")]
        goal_radius: f64,
        #[serde(default = "default_lookahead_distance_m")]
        lookahead_distance_m: f64,
        #[serde(default)]
        state_source: PathFollowingStateSourceConfig,
    },
    PurePursuit {
        max_speed_m_s: f64,
        min_speed_m_s: f64,
        #[serde(default = "default_lookahead_distance_m")]
        lookahead_distance_m: f64,
        #[serde(default)]
        lookahead_time_s: Option<f64>,
        #[serde(default = "default_goal_radius")]
        goal_radius: f64,
        #[serde(default = "default_max_lateral_acceleration")]
        max_lateral_acceleration: f64,
        #[serde(default)]
        state_source: PathFollowingStateSourceConfig,
    },
}

impl PathFollowingConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            PathFollowingConfig::SteeringPid { .. } => "SteeringPid",
            PathFollowingConfig::PurePursuit { .. } => "PurePursuit",
        }
    }
    pub fn state_source(&self) -> PathFollowingStateSourceConfig {
        match self {
            PathFollowingConfig::SteeringPid { state_source, .. } => *state_source,
            PathFollowingConfig::PurePursuit { state_source, .. } => *state_source,
        }
    }
}
