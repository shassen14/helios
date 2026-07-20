use serde::Deserialize;

use super::pose::Pose;

/// Discriminated union of all sensor kinds.
/// The `tag = "kind"` tells Serde to look for a `kind = "..."` field in the TOML.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum SensorConfig {
    Imu(ImuConfig),
    Gps(GpsConfig),
    Lidar(LidarConfig),
    Magnetometer(MagnetometerConfig),
}

impl SensorConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            SensorConfig::Imu(_) => "Imu",
            SensorConfig::Gps(_) => "Gps",
            SensorConfig::Lidar(_) => "Lidar",
            SensorConfig::Magnetometer(_) => "Magnetometer",
        }
    }
}

/// Configuration parameters for a simulated IMU sensor.
///
/// An IMU chip always produces two physical quantities: linear acceleration
/// (`LinearAcceleration3D`) and angular velocity (`AngularVelocity3D`). If the
/// chip also has an onboard magnetometer, add a separate `Magnetometer` entry
/// to the sensor suite — that quantity is independently configured and spawns
/// its own sensor with its own forward model.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuConfig {
    pub name: String,
    pub rate: f64,
    #[serde(default)]
    pub transform: Pose,
    /// Constant accelerometer offset along the sensor's [X, Y, Z] FLU axes,
    /// in m/s². Defaults to zero (a perfectly calibrated sensor).
    #[serde(default)]
    pub accel_bias: [f64; 3],
    /// Per-axis noise standard deviation for the accelerometer, in m/s².
    #[serde(default)]
    pub accel_noise_stddev: [f64; 3],
    /// Constant gyroscope offset along the sensor's [X, Y, Z] FLU axes, in
    /// rad/s. This is the drift rate a stationary gyro reports; it is the
    /// dominant error term in dead reckoning. Defaults to zero.
    #[serde(default)]
    pub gyro_bias: [f64; 3],
    /// Per-axis noise standard deviation for the gyroscope, in rad/s.
    #[serde(default)]
    pub gyro_noise_stddev: [f64; 3],

    /// Bus channel for `Vec<SensorReading<LinearAcceleration3D>>`. Must match the
    /// accelerometer `input_channel` in the estimator's aiding config. Distinct
    /// from `gyro_channel` — one IMU publishes its two quantities on two separate
    /// channels. The name disambiguates multiple IMUs on a single agent's bus; it
    /// need not be unique across agents, since each agent owns its own bus.
    pub accel_channel: String,
    /// Bus channel for `Vec<SensorReading<AngularVelocity3D>>`. Must match the
    /// gyroscope `input_channel` in the estimator's aiding config. Same
    /// per-agent uniqueness rule as `accel_channel`.
    pub gyro_channel: String,
}

impl ImuConfig {
    pub fn get_name(&self) -> &str {
        &self.name
    }
    pub fn get_rate(&self) -> f64 {
        self.rate
    }
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
    /// The accelerometer and gyroscope noise standard deviations, in that order.
    pub fn get_noise_stddevs(&self) -> ([f64; 3], [f64; 3]) {
        (self.accel_noise_stddev, self.gyro_noise_stddev)
    }
    /// The accelerometer and gyroscope biases, in that order.
    pub fn get_biases(&self) -> ([f64; 3], [f64; 3]) {
        (self.accel_bias, self.gyro_bias)
    }

    pub fn get_accel_channel(&self) -> &str {
        &self.accel_channel
    }
    pub fn get_gyro_channel(&self) -> &str {
        &self.gyro_channel
    }
}

/// Configuration parameters for a simulated GPS sensor.
#[derive(Debug, Clone, Deserialize)]
pub struct GpsConfig {
    pub name: String,
    pub rate: f64,
    /// Bus channel name for `Vec<SensorReading<GpsPosition>>` published to the pipeline.
    /// Must match the `input_channel` in the estimator's aiding config.
    pub channel: String,
    #[serde(default)]
    pub transform: Pose,

    /// Constant measurement offset in [East, North, Up] axes, in meters.
    #[serde(default)]
    pub bias: [f64; 3],

    /// Standard deviation of noise in [East, North, Up] axes, in meters.
    #[serde(default)]
    pub noise_stddev: [f64; 3],
}

impl GpsConfig {
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
}

/// Configuration parameters for a simulated 3-axis magnetometer.
#[derive(Debug, Clone, Deserialize)]
pub struct MagnetometerConfig {
    pub name: String,
    pub rate: f64,
    #[serde(default)]
    pub transform: Pose,
    /// Constant measurement offset along the sensor's [X, Y, Z] FLU axes, in
    /// µT. This models hard-iron distortion — the field of ferrous material
    /// mounted near the sensor — which is fixed in the *sensor* frame and so
    /// rotates with the vehicle, unlike the world's reference field. Defaults
    /// to zero.
    #[serde(default)]
    pub bias: [f64; 3],
    /// Standard deviation of noise along the sensor's [X, Y, Z] FLU axes, in µT.
    #[serde(default)]
    pub noise_stddev: [f64; 3],
    /// Bus channel name for `Vec<SensorReading<MagneticField3D>>` published to the pipeline.
    /// Must match the `input_channel` in the estimator's aiding config.
    pub channel: String,
}

impl MagnetometerConfig {
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
    pub fn get_rate(&self) -> f64 {
        self.rate
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum LidarConfig {
    // The geometry and noise fields stay f32: they parameterize a raycasting
    // model executed against the physics engine, whose spatial queries are
    // f32-native. Only `rate` is f64 — it feeds a Duration, like every other
    // sensor's rate.
    Lidar2D {
        rate: f64,
        #[serde(default)]
        transform: Pose,
        max_range: f32,
        /// Horizontal field of view in degrees.
        horizontal_fov: f32,
        horizontal_beams: u32,
        #[serde(default)]
        range_noise_stddev: f32,
        #[serde(default)]
        debug_visuals: bool,
        /// Bus channel name for `Vec<SensorReading<PointCloud2D>>` published to the pipeline.
        channel: String,
    },
    Lidar3D {
        rate: f64,
        #[serde(default)]
        transform: Pose,
        max_range: f32,
        /// Horizontal field of view in degrees.
        horizontal_fov: f32,
        horizontal_beams: u32,
        /// Vertical field of view in degrees.
        vertical_fov: f32,
        vertical_beams: u32,
        #[serde(default)]
        range_noise_stddev: f32,
        #[serde(default)]
        debug_visuals: bool,
        /// Bus channel name for `Vec<SensorReading<PointCloud3D>>` published to the pipeline.
        channel: String,
    },
}

impl LidarConfig {
    pub fn get_rate(&self) -> f64 {
        match self {
            LidarConfig::Lidar2D { rate, .. } => *rate,
            LidarConfig::Lidar3D { rate, .. } => *rate,
        }
    }

    pub fn get_relative_pose(&self) -> Pose {
        match self {
            LidarConfig::Lidar2D { transform, .. } => *transform,
            LidarConfig::Lidar3D { transform, .. } => *transform,
        }
    }

    pub fn get_debug_visuals_flag(&self) -> bool {
        match self {
            LidarConfig::Lidar2D { debug_visuals, .. } => *debug_visuals,
            LidarConfig::Lidar3D { debug_visuals, .. } => *debug_visuals,
        }
    }

    pub fn get_channel(&self) -> &str {
        match self {
            LidarConfig::Lidar2D { channel, .. } => channel.as_str(),
            LidarConfig::Lidar3D { channel, .. } => channel.as_str(),
        }
    }
}
