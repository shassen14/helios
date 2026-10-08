//! [`IntegratedImuConfig`] — the `[dynamics]` sub-table of the `IntegratedImu`
//! kind, and its prior defaults.

use crate::nodes::estimation::gravity::default_gravity_enu;

use serde::{Deserialize, Serialize};

/// Default prior std dev on position (m): large, for a cold start whose pose
/// is unknown until GNSS arrives.
pub(crate) const DEFAULT_POSITION_UNCERTAINTY_M: f64 = 1000.0;
/// Default prior std dev on velocity (m/s).
pub(crate) const DEFAULT_VELOCITY_UNCERTAINTY_MPS: f64 = 1.0;
/// Default prior std dev on attitude (degrees): any heading.
pub(crate) const DEFAULT_ORIENTATION_UNCERTAINTY_DEG: f64 = 180.0;
/// Default prior std dev on accelerometer bias (m/s²).
pub(crate) const DEFAULT_ACCEL_BIAS_UNCERTAINTY_MPS2: f64 = 0.1;
/// Default prior std dev on gyro bias (rad/s).
pub(crate) const DEFAULT_GYRO_BIAS_UNCERTAINTY_RADPS: f64 = 0.01;

/// The `[dynamics]` sub-table of the `IntegratedImu` kind.
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct IntegratedImuConfig {
    /// The filter's believed gravity, world ENU `[east, north, up]` (m/s²).
    #[serde(default = "default_gravity_enu")]
    pub(super) gravity_enu: [f64; 3],
    /// Velocity random walk: accelerometer white-noise std dev (m/s²/√Hz).
    pub(super) accel_noise_stddev: f64,
    /// Angle random walk: gyro white-noise std dev (rad/s/√Hz).
    pub(super) gyro_noise_stddev: f64,
    /// Accelerometer bias instability std dev (m/s²/√Hz); the bias block's Q.
    pub(super) accel_bias_instability: f64,
    /// Gyro bias instability std dev (rad/s/√Hz); the bias block's Q.
    pub(super) gyro_bias_instability: f64,
    /// The prior's std dev per block; squared onto P₀.
    #[serde(default)]
    pub(super) initial_uncertainty: InitialUncertaintyConfig,
    /// Channel the predict step reads `Vec<SensorReading<Acceleration>>` from.
    pub(super) accel_channel: String,
    /// Channel the predict step reads `Vec<SensorReading<AngularRate>>` from.
    pub(super) gyro_channel: String,
}

/// Prior std dev on each INS block: the state's P₀, which the model bakes
/// into its schema.
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct InitialUncertaintyConfig {
    #[serde(default = "default_position_m")]
    pub(super) position_m: f64,
    #[serde(default = "default_velocity_mps")]
    pub(super) velocity_mps: f64,
    /// Degrees in config; converted to radians on load.
    #[serde(default = "default_orientation_deg")]
    pub(super) orientation_deg: f64,
    /// Distinct from `accel_bias_instability`, which is the block's Q.
    #[serde(default = "default_accel_bias_mps2")]
    pub(super) accel_bias_mps2: f64,
    /// Too large seeds attitude error outside the filter's linear regime.
    #[serde(default = "default_gyro_bias_radps")]
    pub(super) gyro_bias_radps: f64,
}

impl Default for InitialUncertaintyConfig {
    fn default() -> Self {
        Self {
            position_m: DEFAULT_POSITION_UNCERTAINTY_M,
            velocity_mps: DEFAULT_VELOCITY_UNCERTAINTY_MPS,
            orientation_deg: DEFAULT_ORIENTATION_UNCERTAINTY_DEG,
            accel_bias_mps2: DEFAULT_ACCEL_BIAS_UNCERTAINTY_MPS2,
            gyro_bias_radps: DEFAULT_GYRO_BIAS_UNCERTAINTY_RADPS,
        }
    }
}

fn default_position_m() -> f64 {
    DEFAULT_POSITION_UNCERTAINTY_M
}

fn default_velocity_mps() -> f64 {
    DEFAULT_VELOCITY_UNCERTAINTY_MPS
}

fn default_orientation_deg() -> f64 {
    DEFAULT_ORIENTATION_UNCERTAINTY_DEG
}

fn default_accel_bias_mps2() -> f64 {
    DEFAULT_ACCEL_BIAS_UNCERTAINTY_MPS2
}

fn default_gyro_bias_radps() -> f64 {
    DEFAULT_GYRO_BIAS_UNCERTAINTY_RADPS
}
