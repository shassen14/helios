use serde::Deserialize;

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
