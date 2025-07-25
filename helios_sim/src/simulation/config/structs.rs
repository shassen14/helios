// helios_sim/src/simulation/config/structs.rs

use bevy::prelude::{Resource, Transform};
use figment::value::Value;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serde::Deserialize;
use std::{collections::HashMap, path::PathBuf};

use crate::simulation::utils::serde_helpers;

// =========================================================================
// == Top-Level Configuration Resource ==
// =========================================================================

/// # SimulationConfig
/// The primary Bevy resource holding all configuration for a simulation run.
/// This struct is the root of the data parsed from a `scenario.toml` file.
#[derive(Resource, Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)] // Fail if the TOML has fields not in our struct
pub struct ScenarioConfig {
    #[serde(default)] // Use default if the [simulation] section is missing
    pub simulation: Simulation,

    #[serde(default)]
    pub world: World,

    // The TOML has `[[agents]]`, which becomes a Vec of AgentConfig structs.
    #[serde(default)]
    pub agents: Vec<AgentConfig>,
}

// Temporary, helper struct for the initial file loading step,
// because the `agents` field in the TOML is a list of raw Values.
#[derive(Deserialize)]
pub struct RawScenarioConfig {
    #[serde(default)]
    pub simulation: Simulation,
    #[serde(default)]
    pub world: World,
    #[serde(default)]
    pub agents: Vec<Value>,
}

// =========================================================================
// == Configuration Sub-Structs ==
// These map directly to the sections in your scenario.toml file.
// =========================================================================

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Simulation {
    /// Optional seed for the pseudo-random number generator for determinism.
    pub seed: Option<u64>,
    /// Duration of the simulation in seconds.
    pub duration_seconds: f32,
    /// A list of topic names to log at the end of the run.
    #[serde(default)]
    pub log_topics: Vec<String>,
}

impl Default for Simulation {
    fn default() -> Self {
        Self {
            seed: None,
            duration_seconds: 60.0,
            log_topics: Vec::new(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct World {
    /// Path to the glTF file representing the static world map.
    pub map_file: PathBuf,
    /// Global gravity vector in m/s^2.
    pub gravity: [f32; 3],
}

impl Default for World {
    fn default() -> Self {
        Self {
            map_file: "assets/maps/default.gltf".into(),
            gravity: [0.0, -9.81, 0.0],
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct AgentConfig {
    pub name: String,
    pub starting_pose: Pose,
    pub goal_pose: Pose,
    pub vehicle: Vehicle,
    #[serde(default)]
    pub sensors: HashMap<String, SensorConfig>,
    #[serde(default)]
    pub autonomy_stack: AutonomyStack,
}

// =========================================================================
// == Helper Structs for Nested Configuration ==
// =========================================================================

#[derive(Deserialize, Debug, Clone, Copy, Default)]
pub struct Pose {
    // Point to the module, not the function
    #[serde(with = "serde_helpers::vec3_f64_from_f32_array", default)]
    pub translation: Vector3<f64>,

    // --- CORRECTED ATTRIBUTE ---
    #[serde(with = "serde_helpers::quat_f64_from_euler_deg_f32", default)]
    pub rotation: UnitQuaternion<f64>,
}

impl Pose {
    pub fn to_isometry(&self) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::from(self.translation), self.rotation)
    }

    pub fn to_bevy_transform(&self) -> Transform {
        let t = self.translation;
        let r = self.rotation.coords;
        Transform::from_xyz(t.x as f32, t.y as f32, t.z as f32).with_rotation(
            bevy::prelude::Quat::from_xyzw(r.x as f32, r.y as f32, r.z as f32, r.w as f32),
        )
    }
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")] // This tells serde to use the "kind" field to decide which enum variant to parse
#[serde(rename_all = "PascalCase")] // e.g., "Ackermann" in TOML maps to `Ackermann` variant
pub enum Vehicle {
    Ackermann {
        wheelbase: f32,
        max_steering_angle: f32, // in degrees
        max_steering_rate: f32,  // in deg/sec
    },
    Quadcopter {
        // Parameters specific to a quadcopter
        rotor_thrust_coefficient: f32,
        rotor_drag_coefficient: f32,
    },
}

impl Vehicle {
    pub fn get_kind_str(&self) -> &str {
        match self {
            Vehicle::Ackermann { .. } => "Ackermann",
            Vehicle::Quadcopter { .. } => "Quadcopter",
        }
    }
}

// =========================================================================
// == Sensors ==
// =========================================================================

// This enum can represent ANY sensor that might appear in the config list.
// The `tag = "kind"` tells Serde to look for a `kind = "..."` field in the TOML
// to decide which variant to parse.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")] // "Imu" in TOML maps to Imu variant
pub enum SensorConfig {
    Imu(ImuConfig), // This variant holds the existing ImuConfig struct
    Gps(GpsConfig),
    Lidar(LidarConfig),
    Magnetometer(MagnetometerConfig),
    // When you add a camera, you'll add a new variant here:
    // Camera(CameraConfig),
}

impl SensorConfig {
    // Helper to get the string identifier for the registry
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
        // Be explicit about what the noise values mean
        #[serde(default)]
        accel_noise_stddev: [f32; 3], // [x, y, z]
        #[serde(default)]
        gyro_noise_stddev: [f32; 3], // [roll, pitch, yaw]
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
        mag_noise_stddev: [f32; 3], // The new field
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
    /// A unique name for this sensor instance (e.g., "primary_gps").
    pub name: String,

    /// The rate at which the GPS produces measurements, in Hz.
    pub rate: f32,

    /// The static transform (position and orientation) of the GPS antenna
    /// relative to its parent frame.
    #[serde(default)]
    pub transform: Pose,

    /// The standard deviation of the noise to be added to the measurement,
    /// in meters, for the [East, North, Up] axes respectively.
    #[serde(default)]
    pub noise_stddev: [f32; 3],
}

// We add helper methods for convenient and consistent access.
impl GpsConfig {
    /// Returns the relative pose of the GPS antenna.
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
}

/// Configuration parameters for a simulated 3-axis magnetometer.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MagnetometerConfig {
    /// A unique name for this sensor instance.
    pub name: String,

    /// The rate at which the magnetometer produces measurements, in Hz.
    pub rate: f32,

    /// The static transform (position and orientation) of the sensor
    /// relative to its parent frame (the agent's body).
    #[serde(default)]
    pub transform: Pose,

    /// The standard deviation of the noise to be added to the measurement,
    /// in micro-teslas (uT) or other unit, for the [X, Y, Z] body axes.
    #[serde(default)]
    pub noise_stddev: [f32; 3],
}

// Add helper methods for consistency
impl MagnetometerConfig {
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
}

// --- LiDAR ---
// LidarConfig is also an enum to handle different LiDAR kinds.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum LidarConfig {
    Lidar2D {
        name: String,
        rate: f32,
        #[serde(default)]
        transform: Pose,
        range: f32,
        // ... other 2D-specific params ...
    },
    Lidar3D {
        name: String,
        rate: f32,
        #[serde(default)]
        transform: Pose,
        vertical_fov: f32,
        num_channels: u32,
        // ... other 3D-specific params ...
    },
}
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

// --- The WorldModelConfig Enum ---
// This enum defines the mutually exclusive ways to configure the world model.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type")] // Look for a field named "type".
#[serde(rename_all = "PascalCase")] // "Separate" in TOML maps to this.
pub enum WorldModelConfig {
    CombinedSlam {
        slam: SlamConfig,
    },
    Separate {
        estimator: Option<EstimatorConfig>,
        mapper: Option<MapperConfig>,
    },
}

// Default to the separate configuration with no modules active.
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
// The "kind" field in TOML determines the variant
#[serde(rename_all = "PascalCase")]
pub enum EstimatorConfig {
    Ekf(EkfConfig), // We wrap a specific struct for clarity
    Ukf(UkfConfig), // Future-proofing
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
    // This field now becomes an enum that bundles the model type
    // with its specific tuning parameters.
    pub dynamics: EkfDynamicsConfig,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type")] // Use the 'type' field in TOML to select the variant.
#[serde(rename_all = "PascalCase")]
pub enum EkfDynamicsConfig {
    // This variant is for your existing IMU-based model.
    IntegratedImu(ImuProcessNoiseConfig),

    // This variant is for a future Ackermann odometry model.
    AckermannOdometry(AckermannProcessNoiseConfig),

    // This variant is for a future quadcopter model.
    Quadcopter(QuadcopterProcessNoiseConfig),
}

// A struct holding only the process noise parameters for the IMU model.
#[derive(Debug, Deserialize, Clone)]
pub struct ImuProcessNoiseConfig {
    /// Velocity Random Walk (N): The standard deviation of the white noise on the accelerometer.
    /// This is your accel_noise_stddev from the sensor config. Units: m/s² / sqrt(Hz).
    pub accel_noise_stddev: f64,
    /// Angle Random Walk (N_g): The standard deviation of the white noise on the gyroscope.
    /// This is your gyro_noise_stddev. Units: rad/s / sqrt(Hz).
    pub gyro_noise_stddev: f64,
    /// Standard deviation representing the instability of the accelerometer bias.
    /// Higher values mean the filter trusts its bias estimate less and adapts more quickly.
    /// Units: m/s^2 / sqrt(Hz)
    pub accel_bias_instability: f64,
    /// Standard deviation representing the instability of the gyroscope bias.
    /// Units: rad/s / sqrt(Hz)
    pub gyro_bias_instability: f64,
}

// A struct holding only the process noise parameters for an Ackermann model.
#[derive(Debug, Deserialize, Clone)]
pub struct AckermannProcessNoiseConfig {
    // Uncertainty in forward velocity, e.g., from wheel slip.
    pub velocity_stddev: f64,
    // Uncertainty in how much the robot turns, e.g., from wheel slip during turns.
    pub yaw_rate_stddev: f64,
}

// A struct holding only the process noise parameters for a quadcopter model.
#[derive(Debug, Deserialize, Clone)]
pub struct QuadcopterProcessNoiseConfig {
    // Uncertainty in forces acting on the drone.
    pub force_stddev: f64,
    // Uncertainty in torques acting on the drone.
    pub torque_stddev: f64,
}

// Example for a future UKF config
#[derive(Debug, Deserialize, Clone)]
pub struct UkfConfig {
    pub name: String,
    pub rate: f32,
    pub alpha: f64,
    pub beta: f64,
    pub kappa: f64,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum MapperConfig {
    None, // An explicit option for no mapper
    OccupancyGrid2D { rate: f32, resolution: f32 },
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum SlamConfig {
    EkfSlam(EkfSlamConfig),
    FactorGraphSlam(FactorGraphSlamConfig),
}

// Define config structs for each SLAM kind
#[derive(Debug, Deserialize, Clone)]
pub struct EkfSlamConfig {/* TODO: ... params for EKF-SLAM ... */}

#[derive(Debug, Deserialize, Clone)]
pub struct FactorGraphSlamConfig {/* TODO: ... params for FG-SLAM ... */}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum PlannerConfig {
    AStar { rate: f32 },
    // RrtStar { rate: f32 },
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
    // Mpc { rate: f32, horizon: u32 },
}
