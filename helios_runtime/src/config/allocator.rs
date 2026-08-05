use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum AllocatorConfig {
    KinematicAckermann {
        wheelbase: f64,
        wheel_radius: f64,
        drive: String,
        steer: String,
    },
}

impl AllocatorConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            AllocatorConfig::KinematicAckermann { .. } => "KinematicAckermann",
        }
    }
}
