//! [`AStarPlannerConfig`] — the `[nodes.<name>]` section of an `AStar` entry.

use serde::{Deserialize, Serialize};

/// An A* planner over one map. The planner publishes its path on a channel
/// named after the node, so its name is the table key and has no field here.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct AStarPlannerConfig {
    /// Replanning rate in Hz.
    pub(crate) rate: f32,
    /// Distance from the goal, in meters, at which the planner reports it
    /// reached.
    #[serde(default = "default_arrival_tolerance_m")]
    pub(crate) arrival_tolerance_m: f32,
    /// Occupancy value at or above which a cell is an obstacle.
    #[serde(default = "default_occupancy_threshold")]
    pub(crate) occupancy_threshold: u8,
    /// Most cells one search expands before giving up.
    #[serde(default = "default_max_search_depth")]
    pub(crate) max_search_depth: usize,
    /// Smooth the searched path before publishing it.
    #[serde(default)]
    pub(crate) enable_path_smoothing: bool,
    /// Replan when the robot strays more than `deviation_tolerance_m` from the
    /// current path.
    #[serde(default)]
    pub(crate) replan_on_path_deviation: bool,
    /// Distance from the path, in meters, that triggers a replan.
    #[serde(default = "default_deviation_tolerance_m")]
    pub(crate) deviation_tolerance_m: f32,
    /// Channel the planner reads `MapData` from: the name of the map node that
    /// publishes it.
    pub(crate) map_channel: String,
    /// Channel the planner reads its `PlannerGoal` from. The host writes the
    /// goal to the same name.
    #[serde(default = "default_goal_channel")]
    pub(crate) goal_channel: String,
}

fn default_arrival_tolerance_m() -> f32 {
    1.5
}

fn default_occupancy_threshold() -> u8 {
    180
}

fn default_max_search_depth() -> usize {
    50_000
}

fn default_deviation_tolerance_m() -> f32 {
    3.0
}

fn default_goal_channel() -> String {
    "mission".to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    const SECTION: &str = "rate = 5.0\nmap_channel = \"local\"";

    // The registry relies on `deny_unknown_fields` to turn a misspelled key
    // into an error instead of a silently ignored one. `level`, the key this
    // section used before `map_channel`, is rejected the same way.
    #[test]
    fn a_misspelled_or_retired_key_is_rejected() {
        let misspelled = format!("{SECTION}\ngoal_chanel = \"mission\"");
        assert!(toml::from_str::<AStarPlannerConfig>(&misspelled).is_err());

        let retired = format!("{SECTION}\nlevel = \"local\"");
        assert!(toml::from_str::<AStarPlannerConfig>(&retired).is_err());
    }

    // The map channel has no default: a planner names the map it plans over.
    #[test]
    fn map_channel_is_required() {
        assert!(toml::from_str::<AStarPlannerConfig>("rate = 5.0").is_err());
    }

    #[test]
    fn goal_channel_defaults_to_mission_and_is_read_when_present() {
        let default: AStarPlannerConfig = toml::from_str(SECTION).expect("a valid section");
        assert_eq!(default.goal_channel, "mission");

        let renamed: AStarPlannerConfig =
            toml::from_str(&format!("{SECTION}\ngoal_channel = \"waypoints\""))
                .expect("a valid section");
        assert_eq!(renamed.goal_channel, "waypoints");
    }
}
