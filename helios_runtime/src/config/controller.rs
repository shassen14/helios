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
    Pid {
        rate: f32,
        kp: f32,
        ki: f32,
        kd: f32,
        #[serde(default)]
        state_source: ControllerStateSourceConfig,
    },
    Lqr {
        /// Flat, row-major K matrix (control_dim × state_dim elements).
        gain_matrix: Vec<f64>,
        state_dim: usize,
        control_dim: usize,
        #[serde(default)]
        u_min: Vec<f64>,
        #[serde(default)]
        u_max: Vec<f64>,
        #[serde(default)]
        state_source: ControllerStateSourceConfig,
    },
    FeedforwardPid {
        /// Key into `dynamics_factories` — must match a registered ControlDynamics name.
        dynamics_key: String,
        kp: Vec<f64>,
        ki: Vec<f64>,
        kd: Vec<f64>,
        #[serde(default)]
        u_min: Vec<f64>,
        #[serde(default)]
        u_max: Vec<f64>,
        /// Indices of state vector components that each PID channel tracks.
        #[serde(default)]
        controlled_indices: Vec<usize>,
        #[serde(default)]
        state_source: ControllerStateSourceConfig,
    },
    SteeringPid {
        /// Constant forward speed (m/s) output on the vx channel.
        cruise_speed: f64,
        kp: f64,
        ki: f64,
        kd: f64,
        #[serde(default = "default_goal_radius")]
        goal_radius: f64,
        #[serde(default)]
        state_source: ControllerStateSourceConfig,
    },
    /// Passes Vx and Wz directly from the PathFollower reference to BodyVelocity.
    /// Use this when a PathFollower (e.g., PurePursuit) has already computed
    /// velocity commands and no additional feedback is needed.
    DirectVelocity {
        #[serde(default)]
        state_source: ControllerStateSourceConfig,
    },
}

fn default_goal_radius() -> f64 {
    3.0
}

impl ControllerConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            ControllerConfig::Pid { .. } => "Pid",
            ControllerConfig::Lqr { .. } => "Lqr",
            ControllerConfig::FeedforwardPid { .. } => "FeedforwardPid",
            ControllerConfig::SteeringPid { .. } => "SteeringPid",
            ControllerConfig::DirectVelocity { .. } => "DirectVelocity",
        }
    }

    pub fn state_source(&self) -> ControllerStateSourceConfig {
        match self {
            ControllerConfig::Pid { state_source, .. } => *state_source,
            ControllerConfig::Lqr { state_source, .. } => *state_source,
            ControllerConfig::FeedforwardPid { state_source, .. } => *state_source,
            ControllerConfig::SteeringPid { state_source, .. } => *state_source,
            ControllerConfig::DirectVelocity { state_source, .. } => *state_source,
        }
    }
}
