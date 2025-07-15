// helios_core/src/messages.rs

use crate::types::FrameHandle;
use nalgebra::{Isometry3, Matrix6, Vector3, Vector6}; // Example static vectors

/// A rich, self-describing container for sensor data.
#[derive(Clone, Debug)]
pub enum MeasurementData {
    Imu6Dof(Vector6<f64>),
    Imu9Dof {
        base_data: Vector6<f64>,
        magnetic_field: Vector3<f64>,
    },
    GpsPosition(Vector3<f64>),
    // ...
}

impl MeasurementData {
    /// Returns a view of the primary data vector as a slice, if applicable.
    ///
    /// This is useful for generic algorithms that need to convert the measurement
    /// into a `DVector` without knowing the specific type.
    /// Note: For composite types like `Imu9Dof`, this only returns the primary
    /// motion data (accel/gyro).
    pub fn as_primary_slice(&self) -> Option<&[f64]> {
        match self {
            MeasurementData::Imu6Dof(v) => Some(v.as_slice()),
            MeasurementData::Imu9Dof { base_data, .. } => Some(base_data.as_slice()),
            MeasurementData::GpsPosition(v) => Some(v.as_slice()),
            // MeasurementData::GpsPositionVelocity(v) => Some(v.as_slice()),
            // Add other cases here as you add new variants.
        }
    }
}

/// The generic event carrying measurement data.
/// This replaces the old MeasurementEvent.
#[derive(Clone, Debug)]
pub struct MeasurementMessage {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub timestamp: f64,
    pub data: MeasurementData,
}

// And our universal input packet also fits perfectly here.
use crate::frames::FrameAwareState;
use crate::types::Control;

pub enum ModuleInput<'a> {
    TimeStep { dt: f64, current_time: f64 },
    Control { u: &'a Control },
    Measurement { message: &'a MeasurementMessage },
    PoseUpdate { pose: &'a FrameAwareState },
}

/// The primary output of the estimator. Represents the robot's full dynamic state.
#[derive(Clone, Debug)]
pub struct Odometry {
    pub timestamp: f64,
    /// The agent's estimated pose (position and orientation) in the world frame.
    pub pose: Isometry3<f64>,
    /// The agent's estimated linear and angular velocities in its own body frame.
    pub velocity_body: Vector6<f64>, // [vx, vy, vz, wx, wy, wz]

    /// The agent's estimated linear acceleration in its own body frame.
    /// This is the "true" coordinate acceleration (IMU reading with bias and gravity removed).
    pub linear_acceleration_body: Vector3<f64>,

    /// The agent's estimated angular acceleration in its own body frame.
    pub angular_acceleration_body: Vector3<f64>,

    // --- Covariances ---
    pub pose_covariance: Matrix6<f64>,
    pub velocity_covariance: Matrix6<f64>,
    // You could also add covariances for acceleration if the filter calculates them.
}
