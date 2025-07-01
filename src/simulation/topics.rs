// src/simulation/topics.rs
use bevy::prelude::*;
use nalgebra::{DMatrix, Isometry3, Vector3};

// --- Events (Transient Messages) ---

/// Topic: /sensor/imu/data
#[derive(Event, Clone, Debug)]
pub struct ImuData {
    pub entity: Entity, // The robot this sensor is on
    pub timestamp: f64,
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
}

/// Topic: /control/applied
/// The actual control input applied after actuator limits.
#[derive(Event, Clone, Debug)]
pub struct AppliedControl {
    pub entity: Entity,
    pub timestamp: f64,
    pub control_vector: nalgebra::DVector<f64>,
}

// --- Components (Latched/Stateful Topics) ---

/// Topic: /state/estimated
/// The public, estimated pose of an agent.
#[derive(Component, Clone, Debug)]
pub struct EstimatedPose {
    pub timestamp: f64,
    pub pose: Isometry3<f64>,
    pub covariance: DMatrix<f64>,
}

/// Topic: /state/ground_truth
/// The perfect, ground truth state. Only for sensors and logging.
#[derive(Component, Clone, Debug)]
pub struct GroundTruthState {
    pub pose: Isometry3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    // Add acceleration if needed by dynamics
}
