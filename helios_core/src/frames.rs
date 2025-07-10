// helios_core/src/frames.rs

use nalgebra::{DMatrix, DVector};
use std::hash::Hash;

// --- A generic, framework-agnostic identifier ---
// It can be a simple integer. On a real robot, this might be a hardware ID.
// In the Bevy sim, we will use the bits of the Entity ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct FrameHandle(pub u64);

impl FrameHandle {
    // A convenience method for use in the Bevy adapter crate.
    #[cfg(feature = "bevy")] // This will only compile if the "bevy" feature is enabled
    pub fn from_entity(entity: bevy_ecs::prelude::Entity) -> Self {
        Self(entity.to_bits())
    }

    #[cfg(feature = "bevy")]
    pub fn to_entity(self) -> bevy_ecs::prelude::Entity {
        bevy_ecs::prelude::Entity::from_bits(self.0)
    }
}

/// A unique, hashable identifier for any coordinate frame in the simulation.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum FrameId {
    /// The global ENU simulation frame. The ultimate source of truth.
    World,
    /// The origin of a rigid body, where dynamics are typically calculated.
    /// Identified by the agent's unique FrameHandle.
    Body(FrameHandle),
    /// The specific origin of a sensor component.
    /// Identified by the sensor's own unique FrameHandle.
    Sensor(FrameHandle),
}

/// An enum that defines every possible variable that can exist in a state vector.
/// The FrameId specifies which frame the variable is expressed in.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum StateVariable {
    // --- Cartesian Position ---
    Px(FrameId),
    Py(FrameId),
    Pz(FrameId),
    // --- Cartesian Velocity ---
    Vx(FrameId),
    Vy(FrameId),
    Vz(FrameId),
    // --- Cartesian Acceleration ---
    Ax(FrameId),
    Ay(FrameId),
    Az(FrameId),
    // --- Orientation (as a quaternion) ---
    // Represents the rotation FROM the first frame TO the second frame.
    // e.g., Qx(from: Body, to: World)
    Qx(FrameId, FrameId),
    Qy(FrameId, FrameId),
    Qz(FrameId, FrameId),
    Qw(FrameId, FrameId),
    // --- Angular Velocity ---
    Wx(FrameId),
    Wy(FrameId),
    Wz(FrameId),
    // --- Angular Acceleration --
    Alphax(FrameId),
    Alphay(FrameId),
    Alphaz(FrameId),
    // --- Magnetic Field ---
    MagX(FrameId),
    MagY(FrameId),
    MagZ(FrameId),
}

/// The "smart" state object used by filters. It bundles the state vector
/// with its schema (the layout), covariance, and timestamp.
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
    /// Creates a new state with a given layout, initializing the vector to zero
    /// (with a valid identity quaternion) and the covariance to a scaled identity matrix.
    pub fn new(layout: Vec<StateVariable>, initial_covariance_val: f64, timestamp: f64) -> Self {
        let dim = layout.len();
        let mut vector = DVector::zeros(dim);

        // Find the quaternion part of the state and initialize it to identity (0,0,0,1).
        // This is critical to prevent NaN values from an invalid zero quaternion.
        for (i, var) in layout.iter().enumerate() {
            // Using `matches!` is a clean way to check the enum variant without caring about its contents.
            if matches!(var, StateVariable::Qw(_, _)) {
                vector[i] = 1.0; // Set the 'w' component to 1.
                                 // Note: This assumes only one quaternion in the state. For more complex
                                 // states, this logic would need to be more robust.
                break;
            }
        }

        Self {
            layout,
            vector,
            covariance: DMatrix::identity(dim, dim) * initial_covariance_val,
            last_update_timestamp: timestamp,
        }
    }

    /// Returns the dimension (number of rows) of the state vector.
    pub fn dim(&self) -> usize {
        self.layout.len()
    }

    /// Finds the index of a specific `StateVariable` in the layout.
    pub fn find_idx(&self, var: &StateVariable) -> Option<usize> {
        self.layout.iter().position(|v| v == var)
    }
}

/// The generic event that all sensors will publish.
/// It is framework-agnostic.
#[derive(Clone, Debug)]
pub struct MeasurementEvent {
    /// The handle of the agent this measurement belongs to.
    pub agent_handle: FrameHandle,
    /// The handle of the sensor that produced this measurement.
    pub sensor_handle: FrameHandle,
    /// The raw measurement vector `z`.
    pub z: DVector<f64>,
    /// The timestamp of the measurement.
    pub timestamp: f64,
}
