use serde::Deserialize;
use std::collections::HashMap;

#[derive(Debug, Deserialize, Default, Clone)]
#[serde(deny_unknown_fields)]
pub struct AutonomyStack {
    #[serde(default)]
    pub world_model: Option<WorldModelConfig>,

    #[serde(default)]
    pub planners: HashMap<String, PlannerConfig>,

    #[serde(default)]
    pub controllers: HashMap<String, ControllerConfig>,
}

/// Mutually exclusive ways to configure the world model.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum WorldModelConfig {
    CombinedSlam { slam: SlamConfig },
    Separate {
        estimator: Option<EstimatorConfig>,
        mapper: Option<MapperConfig>,
    },
}

impl Default for WorldModelConfig {
    fn default() -> Self {
        WorldModelConfig::Separate {
            estimator: None,
            mapper: None,
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind", content = "config")]
#[serde(rename_all = "PascalCase")]
pub enum EstimatorConfig {
    Ekf(EkfConfig),
    Ukf(UkfConfig),
}

impl EstimatorConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            EstimatorConfig::Ekf(_) => "Ekf",
            EstimatorConfig::Ukf(_) => "Ukf",
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct EkfConfig {
    pub dynamics: EkfDynamicsConfig,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum EkfDynamicsConfig {
    IntegratedImu(ImuProcessNoiseConfig),
    AckermannOdometry(AckermannProcessNoiseConfig),
    Quadcopter(QuadcopterProcessNoiseConfig),
}

impl EkfDynamicsConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            EkfDynamicsConfig::IntegratedImu(_) => "IntegratedImu",
            EkfDynamicsConfig::AckermannOdometry(_) => "AckermannOdometry",
            EkfDynamicsConfig::Quadcopter(_) => "Quadcopter",
        }
    }
}

/// Process noise parameters for the IMU-integrated dynamics model.
#[derive(Debug, Deserialize, Clone)]
pub struct ImuProcessNoiseConfig {
    /// Velocity Random Walk: std dev of white noise on accelerometer. Units: m/s² / sqrt(Hz).
    pub accel_noise_stddev: f64,
    /// Angle Random Walk: std dev of white noise on gyroscope. Units: rad/s / sqrt(Hz).
    pub gyro_noise_stddev: f64,
    /// Accelerometer bias instability std dev. Units: m/s² / sqrt(Hz).
    pub accel_bias_instability: f64,
    /// Gyroscope bias instability std dev. Units: rad/s / sqrt(Hz).
    pub gyro_bias_instability: f64,
}

/// Process noise parameters for the Ackermann odometry dynamics model.
#[derive(Debug, Deserialize, Clone)]
pub struct AckermannProcessNoiseConfig {
    /// Uncertainty in forward velocity from wheel slip.
    pub velocity_stddev: f64,
    /// Uncertainty in yaw rate from wheel slip during turns.
    pub yaw_rate_stddev: f64,
}

/// Process noise parameters for the quadcopter dynamics model.
#[derive(Debug, Deserialize, Clone)]
pub struct QuadcopterProcessNoiseConfig {
    pub force_stddev: f64,
    pub torque_stddev: f64,
}

#[derive(Debug, Deserialize, Clone)]
pub struct UkfConfig {
    pub name: String,
    pub rate: f32,
    pub alpha: f64,
    pub beta: f64,
    pub kappa: f64,
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum MapperPoseSourceConfig {
    #[default]
    GroundTruth,
    Estimated,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum MapperConfig {
    None,
    OccupancyGrid2D {
        rate: f32,
        resolution: f32,
        #[serde(default)]
        pose_source: MapperPoseSourceConfig,
    },
}

impl MapperConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            MapperConfig::None => "None",
            MapperConfig::OccupancyGrid2D { .. } => "OccupancyGrid2D",
        }
    }

    /// Returns the update rate for mappers that need a `ModuleTimer`, `None` otherwise.
    pub fn get_timer_rate(&self) -> Option<f32> {
        match self {
            MapperConfig::OccupancyGrid2D { rate, .. } => Some(*rate),
            MapperConfig::None => None,
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum SlamConfig {
    EkfSlam(EkfSlamConfig),
    FactorGraphSlam(FactorGraphSlamConfig),
}

impl SlamConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            SlamConfig::EkfSlam(_) => "EkfSlam",
            SlamConfig::FactorGraphSlam(_) => "FactorGraphSlam",
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct EkfSlamConfig {/* TODO: params for EKF-SLAM */}

#[derive(Debug, Deserialize, Clone)]
pub struct FactorGraphSlamConfig {/* TODO: params for FG-SLAM */}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum PlannerConfig {
    AStar { rate: f32 },
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
