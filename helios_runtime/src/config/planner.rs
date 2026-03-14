use serde::Deserialize;

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
fn default_level() -> String {
    "local".to_string()
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum PlannerConfig {
    AStar {
        rate: f32,
        #[serde(default = "default_arrival_tolerance_m")]
        arrival_tolerance_m: f32,
        #[serde(default = "default_occupancy_threshold")]
        occupancy_threshold: u8,
        #[serde(default = "default_max_search_depth")]
        max_search_depth: usize,
        #[serde(default)]
        enable_path_smoothing: bool,
        #[serde(default)]
        replan_on_path_deviation: bool,
        #[serde(default = "default_deviation_tolerance_m")]
        deviation_tolerance_m: f32,
        #[serde(default = "default_level")]
        level: String,
    },
}

impl PlannerConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            PlannerConfig::AStar { .. } => "AStar",
        }
    }

    pub fn get_level_str(&self) -> &str {
        match self {
            PlannerConfig::AStar { level, .. } => level.as_str(),
        }
    }
}
