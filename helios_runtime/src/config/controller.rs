use serde::Deserialize;

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum ControllerStateSourceConfig {
    #[default]
    Estimated,
    GroundTruth,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum ControllerConfig {
    /// Passes Vx and Wz directly from the PathFollower reference to a BodyTwist.
    /// Use this when a PathFollower (e.g., PurePursuit) has already computed
    /// velocity commands and no additional feedback is needed.
    DirectVelocity {
        #[serde(default)]
        state_source: ControllerStateSourceConfig,
    },
}

impl ControllerConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            ControllerConfig::DirectVelocity { .. } => "DirectVelocity",
        }
    }

    pub fn state_source(&self) -> ControllerStateSourceConfig {
        match self {
            ControllerConfig::DirectVelocity { state_source, .. } => *state_source,
        }
    }
}
