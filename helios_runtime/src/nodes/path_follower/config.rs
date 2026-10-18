//! The `[nodes.<name>]` sections of the path-follower kinds: [`PurePursuitConfig`]
//! and [`SteeringPidConfig`].

use serde::{Deserialize, Serialize};

/// A pure-pursuit follower. It publishes its reference on a channel named after
/// the node, so its name is the table key and has no field here.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct PurePursuitConfig {
    /// Channel the follower reads its `Path` from: the name of the planner node
    /// that publishes it.
    pub(crate) path: String,
    /// Speed cap, in m/s, reached on straights.
    pub(crate) max_speed_m_s: f64,
    /// Speed floor, in m/s, so the car never stalls on a tight curve.
    pub(crate) min_speed_m_s: f64,
    /// Distance ahead on the path, in meters, the follower steers toward.
    #[serde(default = "default_lookahead_distance_m")]
    pub(crate) lookahead_distance_m: f64,
    /// When set, the lookahead grows with speed: this many seconds of travel,
    /// never less than `lookahead_distance_m`.
    #[serde(default)]
    pub(crate) lookahead_time_s: Option<f64>,
    /// Distance from the last waypoint, in meters, at which the goal is reached.
    #[serde(default = "default_goal_radius")]
    pub(crate) goal_radius: f64,
    /// Lateral acceleration cap, in m/s², that limits speed on curves.
    #[serde(default = "default_max_lateral_acceleration")]
    pub(crate) max_lateral_acceleration: f64,
}

/// A follower that closes heading error to a lookahead point with a PID. It
/// publishes its reference on a channel named after the node.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct SteeringPidConfig {
    /// Channel the follower reads its `Path` from: the name of the planner node
    /// that publishes it.
    pub(crate) path: String,
    /// Constant forward speed, in m/s.
    pub(crate) cruise_speed: f64,
    #[serde(default = "default_kp")]
    pub(crate) kp: f64,
    #[serde(default = "default_ki")]
    pub(crate) ki: f64,
    #[serde(default = "default_kd")]
    pub(crate) kd: f64,
    /// Distance from the last waypoint, in meters, at which the goal is reached.
    #[serde(default = "default_goal_radius")]
    pub(crate) goal_radius: f64,
    /// Distance ahead on the path, in meters, the follower steers toward.
    #[serde(default = "default_lookahead_distance_m")]
    pub(crate) lookahead_distance_m: f64,
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pure_pursuit_needs_only_its_path_and_speeds() {
        let config: PurePursuitConfig =
            toml::from_str("path = \"local_path\"\nmax_speed_m_s = 5.0\nmin_speed_m_s = 0.5")
                .expect("a minimal pure-pursuit section parses");

        assert_eq!(config.path, "local_path");
        assert_eq!(config.lookahead_time_s, None);
        assert!((config.goal_radius - default_goal_radius()).abs() < f64::EPSILON);
    }

    #[test]
    fn a_stray_key_is_rejected() {
        // `state_source` belongs to controllers; on a follower it was silently
        // ignored before the section rejected unknown keys.
        let result: Result<PurePursuitConfig, _> = toml::from_str(
            "path = \"local_path\"\nmax_speed_m_s = 5.0\nmin_speed_m_s = 0.5\n\
             state_source = \"ground_truth\"",
        );

        assert!(result.is_err(), "`state_source` is not a follower key");
    }

    #[test]
    fn path_is_required() {
        let result: Result<SteeringPidConfig, _> = toml::from_str("cruise_speed = 2.0");

        assert!(result.is_err(), "a follower without a path must not parse");
    }
}
