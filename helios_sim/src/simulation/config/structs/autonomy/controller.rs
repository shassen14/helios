use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum ControllerConfig {
    Pid {
        rate: f32,
        kp: f32,
        ki: f32,
        kd: f32,
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
    },
}
