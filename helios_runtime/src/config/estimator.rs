use serde::Deserialize;

fn default_gravity_enu() -> [f64; 3] {
    [0.0, 0.0, -9.81]
}

fn default_position_uncertainty_m() -> f64 {
    1000.0
}

fn default_orientation_uncertainty_deg() -> f64 {
    180.0
}

fn default_velocity_uncertainty_mps() -> f64 {
    1.0
}

fn default_accel_bias_uncertainty_mps2() -> f64 {
    0.1
}

fn default_gyro_bias_uncertainty_radps() -> f64 {
    0.01
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
    pub augmentation: Vec<AugmentationConfig>,
    #[serde(default)]
    pub initial_state: EkfInitialStateConfig,
}

/// One online-estimated per-sensor nuisance block appended to the EKF state.
///
/// The filter solves for this parameter alongside the trajectory instead of
/// trusting a fixed factory value — online calibration; the same block, frozen
/// after convergence, is the offline-calibration result.
///
/// `sensor` must name an aiding entry's `input_channel`: that aiding source is
/// what makes the block observable, because its measurement is the only thing
/// that touches these state columns. The two also share a resolved
/// `FrameHandle` — the assembler routes `sensor` through the same handle map the
/// aiding handler uses, so the appended `MagBias` slots carry the exact
/// `FrameId` the measurement model reads back. An augmentation with no matching
/// aiding source is inert (it rides through predict but is never updated); the
/// validator rejects that case rather than let it be a silent no-op.
#[derive(Debug, Deserialize, Clone)]
pub struct AugmentationConfig {
    /// Augmentation kind, matched against the reserved kind strings in
    /// `helios_core::estimation::augmentation` (e.g. `MAGNETOMETER_BIAS`). An
    /// unrecognized value is a build-time error, not a panic.
    pub kind: String,
    /// The aiding `input_channel` whose sensor this block calibrates — the join
    /// key that ties the block's `FrameId::Sensor` to the model that observes it.
    pub sensor: String,
    /// Prior standard deviation on each axis (block units, e.g. µT for
    /// magnetometer bias). Squared onto the diagonal of the block's `P₀`.
    pub init_uncertainty: f64,
    /// Per-axis process-noise standard deviation driving how fast the estimate
    /// may drift; forms the block's `Q`.
    pub random_walk: f64,
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
    /// Prior std dev on the initial velocity estimate (m/s). Kinematic, so it sits
    /// here with position/orientation rather than in a dynamics-specific config.
    /// Squared onto the velocity block's `P₀`.
    #[serde(default = "default_velocity_uncertainty_mps")]
    pub velocity_uncertainty_mps: f64,
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
            velocity_uncertainty_mps: default_velocity_uncertainty_mps(),
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
    /// The filter's *believed* gravity, world ENU `[east, north, up]` (m/s²).
    /// Required by `"accelerometer"`. Defaults to `[0, 0, -9.81]` (Earth, down);
    /// override for non-standard sites (high altitude, other planets, a tilted
    /// local frame). Must match the simulated world's `[world.atmosphere]`
    /// gravity_enu unless the mismatch is the experiment.
    #[serde(default = "default_gravity_enu")]
    pub gravity_enu: [f64; 3],
    /// Expected magnetic field in ENU (µT), the filter's *believed* field.
    /// Required by `"magnetometer"`. Must match the simulated world's
    /// `[world.magnetic_field]` unless the mismatch is the experiment.
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
}

/// Config for the IMU-integrated dynamics model.
///
/// Gravity is a physical constant of the deployment site: a world ENU vector
/// defaulting to `[0, 0, -9.81]` (Earth, down), overridden in the scenario
/// config for non-standard locations (high altitude, other planets, a tilted
/// local frame).
#[derive(Debug, Deserialize, Clone)]
pub struct IntegratedImuConfig {
    /// World-frame gravity `[east, north, up]` (m/s²). See the struct doc.
    #[serde(default = "default_gravity_enu")]
    pub gravity_enu: [f64; 3],
    /// Velocity Random Walk: white noise std dev on accelerometer (m/s²/√Hz).
    pub accel_noise_stddev: f64,
    /// Angle Random Walk: white noise std dev on gyroscope (rad/s/√Hz).
    pub gyro_noise_stddev: f64,
    /// Accelerometer bias instability std dev (m/s²/√Hz).
    pub accel_bias_instability: f64,
    /// Gyroscope bias instability std dev (rad/s/√Hz).
    pub gyro_bias_instability: f64,
    /// Prior std dev on the initial accel-bias estimate at cold start (m/s²).
    /// Squared onto the accel-bias block's `P₀`. Distinct from
    /// `accel_bias_instability`, which is that block's process noise (`Q`).
    #[serde(default = "default_accel_bias_uncertainty_mps2")]
    pub accel_bias_uncertainty_mps2: f64,
    /// Prior std dev on the initial gyro-bias estimate at cold start (rad/s).
    /// Squared onto the gyro-bias block's `P₀`. A too-large value seeds attitude
    /// error outside the filter's linear regime, so it is not a free knob.
    #[serde(default = "default_gyro_bias_uncertainty_radps")]
    pub gyro_bias_uncertainty_radps: f64,
    /// Bus channel the predict step reads `Vec<SensorReading<LinearAcceleration3D>>`
    /// from. Must match the accelerometer channel the host publishes on
    /// (the sensor's `accel_channel` in sim).
    pub accel_channel: String,
    /// Bus channel the predict step reads `Vec<SensorReading<AngularVelocity3D>>`
    /// from. Must match the gyroscope channel the host publishes on
    /// (the sensor's `gyro_channel` in sim).
    pub gyro_channel: String,
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
