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

/// The `kind` tag of [`PathFollowingConfig::PurePursuit`], which is also the key its
/// factory is registered under in the path-follower family.
pub(crate) const PURE_PURSUIT_KIND: &str = "PurePursuit";

/// The `kind` tag of [`PathFollowingConfig::SteeringPid`], which is also the key its
/// factory is registered under in the path-follower family.
pub(crate) const STEERING_PID_KIND: &str = "SteeringPid";

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum PathFollowingConfig {
    SteeringPid {
        /// Channel the follower reads its `Path` from: the name of the
        /// planner node that publishes it.
        path: String,
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
    },
    PurePursuit {
        /// Channel the follower reads its `Path` from: the name of the
        /// planner node that publishes it.
        path: String,
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
    },
}

impl PathFollowingConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            PathFollowingConfig::SteeringPid { .. } => STEERING_PID_KIND,
            PathFollowingConfig::PurePursuit { .. } => PURE_PURSUIT_KIND,
        }
    }

    /// The name of the channel the follower reads its `Path` from.
    pub(crate) fn get_path_str(&self) -> &str {
        match self {
            PathFollowingConfig::SteeringPid { path, .. }
            | PathFollowingConfig::PurePursuit { path, .. } => path.as_str(),
        }
    }
}
