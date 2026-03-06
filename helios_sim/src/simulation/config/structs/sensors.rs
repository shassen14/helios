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

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
#[serde(tag = "type")]
pub enum ImuConfig {
    SixDof {
        name: String,
        rate: f32,
        #[serde(default)]
        transform: Pose,
        #[serde(default)]
        accel_noise_stddev: [f32; 3],
        #[serde(default)]
        gyro_noise_stddev: [f32; 3],
    },
    NineDof {
        name: String,
        rate: f32,
        #[serde(default)]
        transform: Pose,
        #[serde(default)]
        accel_noise_stddev: [f32; 3],
        #[serde(default)]
        gyro_noise_stddev: [f32; 3],
        #[serde(default)]
        mag_noise_stddev: [f32; 3],
    },
}

impl ImuConfig {
    pub fn get_name(&self) -> &str {
        match self {
            ImuConfig::SixDof { name, .. } => name,
            ImuConfig::NineDof { name, .. } => name,
        }
    }
    pub fn get_rate(&self) -> f32 {
        match self {
            ImuConfig::SixDof { rate, .. } => *rate,
            ImuConfig::NineDof { rate, .. } => *rate,
        }
    }
    pub fn get_relative_pose(&self) -> Pose {
        match self {
            ImuConfig::SixDof { transform, .. } => *transform,
            ImuConfig::NineDof { transform, .. } => *transform,
        }
    }
    pub fn get_noise_stddevs(&self) -> ([f32; 3], [f32; 3]) {
        match self {
            ImuConfig::SixDof {
                accel_noise_stddev,
                gyro_noise_stddev,
                ..
            } => (*accel_noise_stddev, *gyro_noise_stddev),
            ImuConfig::NineDof {
                accel_noise_stddev,
                gyro_noise_stddev,
                ..
            } => (*accel_noise_stddev, *gyro_noise_stddev),
        }
    }
}

/// Configuration parameters for a simulated GPS sensor.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GpsConfig {
    pub name: String,
    pub rate: f32,
    #[serde(default)]
    pub transform: Pose,
    /// Standard deviation of noise in [East, North, Up] axes, in meters.
    #[serde(default)]
    pub noise_stddev: [f32; 3],
}

impl GpsConfig {
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
}

/// Configuration parameters for a simulated 3-axis magnetometer.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MagnetometerConfig {
    pub name: String,
    pub rate: f32,
    #[serde(default)]
    pub transform: Pose,
    /// Standard deviation of noise in [X, Y, Z] body axes, in µT.
    #[serde(default)]
    pub noise_stddev: [f32; 3],
}

impl MagnetometerConfig {
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
    pub fn get_rate(&self) -> f32 {
        self.rate
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum LidarConfig {
    Lidar2D {
        rate: f32,
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
    },
    Lidar3D {
        rate: f32,
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
    },
}

impl LidarConfig {
    pub fn get_rate(&self) -> f32 {
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
}
