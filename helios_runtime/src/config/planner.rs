use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum PlannerConfig {
    AStar { rate: f32 },
}

impl PlannerConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            PlannerConfig::AStar { .. } => "AStar",
        }
    }
}
