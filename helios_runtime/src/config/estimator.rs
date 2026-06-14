use serde::Deserialize;

fn default_gravity() -> f64 {
    9.81
}

fn default_position_uncertainty_m() -> f64 {
    1000.0
}

fn default_orientation_uncertainty_deg() -> f64 {
    180.0
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind", content = "config")]
#[serde(rename_all = "PascalCase")]
pub enum EstimatorConfig {
    Ekf(EkfConfig),
    Ukf(UkfConfig),
    MockOracle(MockOracleEstimatorConfig),
}

impl EstimatorConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            EstimatorConfig::Ekf(_) => "Ekf",
            EstimatorConfig::Ukf(_) => "Ukf",
            EstimatorConfig::MockOracle(_) => "MockOracle",
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct EkfConfig {
    pub dynamics: EkfDynamicsConfig,
    #[serde(default)]
    pub aiding: Vec<AidingConfig>,
    #[serde(default)]
    pub initial_state: EkfInitialStateConfig,
}

/// Initial mean and covariance parameters for EKF cold-start.
///
/// When the deployment pose is unknown (hardware cold-start, diffuse
/// initialization), leave x/y/z/heading at their defaults (all zero) and set
/// `position_uncertainty_m` large (default 1000 m). The filter will converge
/// once GPS measurements arrive.
///
/// In simulation, the scenario overrides these fields with the agent's actual
/// spawn pose and tight uncertainty.
#[derive(Debug, Deserialize, Clone)]
pub struct EkfInitialStateConfig {
    #[serde(default)]
    pub x: f64,
    #[serde(default)]
    pub y: f64,
    #[serde(default)]
    pub z: f64,
    #[serde(default)]
    pub heading_deg: f64,
    #[serde(default = "default_position_uncertainty_m")]
    pub position_uncertainty_m: f64,
    #[serde(default = "default_orientation_uncertainty_deg")]
    pub orientation_uncertainty_deg: f64,
}

impl Default for EkfInitialStateConfig {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            heading_deg: 0.0,
            position_uncertainty_m: default_position_uncertainty_m(),
            orientation_uncertainty_deg: default_orientation_uncertainty_deg(),
        }
    }
}

/// One aiding sensor channel fed into a Gaussian estimator.
///
/// `r_diag` lives here as a pragmatic placeholder. R values should ultimately come
/// from the sensor entity's `[sim]` or `[hw]` section so each host carries its
/// own calibrated noise. Migrating to entity-file sourcing requires the sensor
/// entity loading infrastructure to pass resolved values into the assembler —
/// tracked as a follow-up to Step 7c.
#[derive(Debug, Deserialize, Clone)]
pub struct AidingConfig {
    /// Rust type name of the sensor payload — one of the `SensorPayload`
    /// implementors: `"GpsPosition"`, `"LinearAcceleration3D"`,
    /// `"AngularVelocity3D"`, `"MagneticField3D"`, `"GpsVelocity"`.
    pub sensor_payload: String,
    /// Sensor model config, including the registry key and any physical
    /// constants the model needs (gravity for accelerometer, field vector for
    /// magnetometer).
    pub model: SensorModelConfig,
    /// Bus channel to read `Vec<SensorReading<T>>` from.
    /// Example naming convention: `"sensor.gps.primary"`.
    pub input_channel: String,
    /// Diagonal of the measurement noise covariance R. Length must equal the
    /// model's measurement dimension.
    pub r_diag: Vec<f64>,
}

/// Physical measurement model config — the registry key and any world-level
/// constants the model's math requires.
///
/// These values belong in `configs/runtime/catalog/sensor_models/` prefabs
/// and are resolved into this struct by the catalog loader before
/// `build_pipeline()` is called.
#[derive(Debug, Deserialize, Clone)]
pub struct SensorModelConfig {
    /// Registry key, e.g. `"gps_position"`, `"accelerometer"`.
    pub kind: String,
    /// Gravitational acceleration (m/s²). Required by `"accelerometer"`.
    /// Defaults to 9.81; override in deployment config for non-standard sites.
    #[serde(default = "default_gravity")]
    pub gravity: f64,
    /// Expected magnetic field in ENU (nT). Required by `"magnetometer"`.
    pub magnetic_field_enu: Option<[f64; 3]>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum EkfDynamicsConfig {
    IntegratedImu(IntegratedImuConfig),
    AckermannOdometry(AckermannProcessNoiseConfig),
    Quadcopter(QuadcopterProcessNoiseConfig),
}

impl EkfDynamicsConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            EkfDynamicsConfig::IntegratedImu(_) => "IntegratedImu",
            EkfDynamicsConfig::AckermannOdometry(_) => "AckermannOdometry",
            EkfDynamicsConfig::Quadcopter(_) => "Quadcopter",
        }
    }

    /// Gravitational acceleration used by this dynamics model.
    pub(crate) fn gravity(&self) -> f64 {
        match self {
            EkfDynamicsConfig::IntegratedImu(c) => c.gravity,
            EkfDynamicsConfig::AckermannOdometry(_) => default_gravity(),
            EkfDynamicsConfig::Quadcopter(_) => default_gravity(),
        }
    }
}

/// Config for the IMU-integrated dynamics model.
///
/// Gravity is a physical constant of the deployment site; it defaults to
/// 9.81 m/s² and should be overridden in the scenario config for non-standard
/// locations (e.g. high altitude, other planets).
#[derive(Debug, Deserialize, Clone)]
pub struct IntegratedImuConfig {
    #[serde(default = "default_gravity")]
    pub gravity: f64,
    /// Velocity Random Walk: white noise std dev on accelerometer (m/s²/√Hz).
    pub accel_noise_stddev: f64,
    /// Angle Random Walk: white noise std dev on gyroscope (rad/s/√Hz).
    pub gyro_noise_stddev: f64,
    /// Accelerometer bias instability std dev (m/s²/√Hz).
    pub accel_bias_instability: f64,
    /// Gyroscope bias instability std dev (rad/s/√Hz).
    pub gyro_bias_instability: f64,
}

/// Process noise parameters for the Ackermann odometry dynamics model.
#[derive(Debug, Deserialize, Clone)]
pub struct AckermannProcessNoiseConfig {
    pub velocity_stddev: f64,
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

#[derive(Debug, Deserialize, Clone, Default)]
pub struct MockOracleEstimatorConfig {}
