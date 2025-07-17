// src/simulation/core/frames.rs
use bevy::prelude::Entity;
use nalgebra::{DMatrix, DVector};

// A unique, hashable identifier for any coordinate frame in the simulation.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum FrameId {
    /// The global ENU simulation frame. The ultimate source of truth.
    World,
    /// The origin of a rigid body, where dynamics are typically calculated.
    /// Identified by the parent agent's Entity ID.
    Body(Entity),
    /// The specific origin of a sensor component.
    /// Identified by the sensor's own child Entity ID.
    Sensor(Entity),
    // NED and BevyWorld could be added here if you need to frequently
    // transform to/from those specific conventions. For now, this is enough.
}

// An enum that defines every possible variable that can exist in a state vector.
// The FrameId specifies which frame the variable is expressed in.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum StateVariable {
    // Position (e.g., in the World frame)
    Px(FrameId),
    Py(FrameId),
    Pz(FrameId),
    // Velocity (e.g., in the Body frame)
    Vx(FrameId),
    Vy(FrameId),
    Vz(FrameId),
    // Acceleration (e.g., in the Body frame)
    Ax(FrameId),
    Ay(FrameId),
    Az(FrameId),
    // Orientation (as a quaternion, representing rotation from a frame to another)
    Qx(FrameId, FrameId), // e.g., Qx(from: Body, to: World)
    Qy(FrameId, FrameId),
    Qz(FrameId, FrameId),
    Qw(FrameId, FrameId),
    // Angular Velocity (almost always in the Body frame)
    Wx(FrameId),
    Wy(FrameId),
    Wz(FrameId),
    // Angular Acceleration (almost always in the Body frame)
    Alphax(FrameId),
    Alphay(FrameId),
    Alphaz(FrameId),
    // Magnetic Field
    MagX(FrameId),
    MagY(FrameId),
    MagZ(FrameId),
    // Biases (tied to a specific sensor on an entity)
    AccelBiasX(Entity),
    AccelBiasY(Entity),
    AccelBiasZ(Entity),
    GyroBiasX(Entity),
    GyroBiasY(Entity),
    GyroBiasZ(Entity),
}

// --- GENERIC STATE CONTAINER ---
// This is the "smart" state object used by the EKF and other estimators.
// It is NOT a component itself, but will be part of a component.
#[derive(Debug, Clone)]
pub struct FrameAwareState {
    /// The ordered "schema" of the state vector.
    pub layout: Vec<StateVariable>,
    /// The actual numerical data vector `x`.
    pub vector: DVector<f64>,
    /// The covariance matrix `P`.
    pub covariance: DMatrix<f64>,
    /// The timestamp of the last update.
    pub last_update_timestamp: f64,
}

impl FrameAwareState {
    pub fn new(layout: Vec<StateVariable>, initial_covariance_val: f64, timestamp: f64) -> Self {
        let dim = layout.len();
        Self {
            layout,
            vector: DVector::zeros(dim),
            covariance: DMatrix::identity(dim, dim) * initial_covariance_val,
            last_update_timestamp: timestamp,
        }
    }
    pub fn dim(&self) -> usize {
        self.layout.len()
    }
    pub fn find_idx(&self, var: &StateVariable) -> Option<usize> {
        self.layout.iter().position(|v| v == var)
    }
}

// --- GENERIC MEASUREMENT EVENT ---
// This is the generic event that all sensors will publish.
#[derive(bevy::prelude::Event, Clone)]
pub struct MeasurementEvent {
    /// Which agent's EKF should process this?
    pub agent_entity: Entity,
    /// Which sensor entity produced it (so we can find its model)?
    pub sensor_entity: Entity,
    /// The raw measurement vector `z`.
    pub z: DVector<f64>,
    /// The timestamp of the measurement.
    pub timestamp: f64,
}
