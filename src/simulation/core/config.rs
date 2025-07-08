// src/simulation/core/config.rs

use bevy::prelude::Resource;
use serde::Deserialize;
use std::collections::HashMap;
use std::path::PathBuf;

// =========================================================================
// == Top-Level Configuration Resource ==
// =========================================================================

/// # SimulationConfig
/// The primary Bevy resource holding all configuration for a simulation run.
/// This struct is the root of the data parsed from a `scenario.toml` file.
#[derive(Resource, Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)] // Fail if the TOML has fields not in our struct
pub struct SimulationConfig {
    #[serde(default)] // Use default if the [simulation] section is missing
    pub simulation: Simulation,

    #[serde(default)]
    pub world: World,

    // The TOML has `[[agents]]`, which becomes a Vec of AgentConfig structs.
    #[serde(default)]
    pub agents: Vec<AgentConfig>,
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

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AgentConfig {
    pub name: String,
    pub starting_pose: Pose,
    pub goal_pose: Pose,
    pub vehicle: Vehicle,
    #[serde(default)] // The whole sensors section can be optional
    pub sensors: SensorSuiteConfig,
    #[serde(default)]
    pub autonomy_stack: AutonomyStack,
}

// =========================================================================
// == Helper Structs for Nested Configuration ==
// =========================================================================

#[derive(Debug, Deserialize)]
pub struct Pose {
    pub position: [f32; 3],
    #[serde(default)]
    pub orientation_deg: [f32; 3], // Roll, Pitch, Yaw
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type")] // This tells serde to use the "type" field to decide which enum variant to parse
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

#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct SensorSuiteConfig {
    // The key is the sensor's name (e.g., "imu_main", "imu_backup").
    // The value is the configuration for that specific sensor.
    #[serde(default)]
    pub imu: HashMap<String, ImuConfig>,

    #[serde(default)]
    pub gps: HashMap<String, GpsConfig>,

    #[serde(default)]
    pub lidar: HashMap<String, LidarConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
#[serde(tag = "type")]
pub enum ImuConfig {
    SixDof {
        // Changed from "NineDof" to "SixDof" for clarity
        rate: f32,
        // Be explicit about what the noise values mean
        #[serde(default)]
        accel_noise_stddev: [f32; 3], // [x, y, z]
        #[serde(default)]
        gyro_noise_stddev: [f32; 3], // [roll, pitch, yaw]
        frame_id: Option<String>,
    },
    NineDof {
        rate: f32,
        #[serde(default)]
        accel_noise_stddev: [f32; 3],
        #[serde(default)]
        gyro_noise_stddev: [f32; 3],
        #[serde(default)]
        mag_noise_stddev: [f32; 3], // The new field
        frame_id: Option<String>,
    },
}
#[derive(Debug, Deserialize)]
pub struct GpsConfig {
    pub rate: f32,
    #[serde(default)]
    pub noise_stddev: [f32; 3],
    // Optional: specify the TF frame it's attached to.
    frame_id: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
#[serde(tag = "type")]
pub enum LidarConfig {
    Lidar2D {
        rate: f32,
        range: f32,
        #[serde(default)]
        noise_stddev: f32,
        // Optional: specify the TF frame it's attached to.
        frame_id: Option<String>,
    },
}

#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct AutonomyStack {
    #[serde(default)]
    pub estimator: Option<EstimatorConfig>,

    #[serde(default)]
    pub mapper: Option<MapperConfig>,

    #[serde(default)]
    pub planner: Option<PlannerConfig>,

    #[serde(default)]
    pub controller: Option<ControllerConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum EstimatorConfig {
    Ekf { rate: f32 },
    // Ukf { rate: f32 },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum MapperConfig {
    OccupancyGrid2D { rate: f32, resolution: f32 },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
#[serde(rename_all = "PascalCase")]
pub enum PlannerConfig {
    AStar { rate: f32 },
    // RrtStar { rate: f32 },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
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
