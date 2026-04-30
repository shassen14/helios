use crate::{
    sensor_data,
    types::{Control, FrameHandle},
};
use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};
use serde::Serialize;

// =========================================================================
// == Core Message and Data Enums ==
// =========================================================================

/// A rich, self-describing container for all sensor data.
#[derive(Clone, Debug, Serialize)]
pub enum MeasurementData {
    LinearAcceleration(sensor_data::LinearAcceleration3D),
    AngularVelocity(sensor_data::AngularVelocity3D),
    GpsPosition(sensor_data::GpsPosition),
    MagneticField(sensor_data::MagneticField3D),
    PointCloud2D(sensor_data::PointCloud2D),
    PointCloud3D(sensor_data::PointCloud3D),
}

/// The generic message that carries all sensor data through the system.
#[derive(Clone, Debug, Serialize)]
pub struct MeasurementMessage {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub timestamp: f64,
    pub data: MeasurementData,
}

/// The universal input packet for all `StateEstimator` implementations.
pub enum ModuleInput<'a> {
    TimeStep {
        dt: f64,
        current_time: f64,
    },
    Control {
        u: &'a Control,
    },
    Measurement {
        message: &'a MeasurementMessage,
    },
    /// Carries the robot's current ENU pose (e.g. from the odom TF frame).
    /// Mappers use this to recenter the grid; it is decoupled from any specific estimator.
    PoseUpdate {
        pose: Isometry3<f64>,
    },
}

// =========================================================================
// == Public API Messages (Topic Data) ==
// =========================================================================

/// The primary output of the estimator. Represents the robot's full dynamic state.
/// This is the standardized message that planners and controllers will consume.
#[derive(Clone, Debug, Default, Serialize)]
pub struct Odometry {
    pub timestamp: f64,
    pub pose: Isometry3<f64>,
    pub velocity_body: Vector6<f64>,
    pub linear_acceleration_body: Vector3<f64>,
    pub angular_acceleration_body: Vector3<f64>,
    pub pose_covariance: Matrix6<f64>,
    pub velocity_covariance: Matrix6<f64>,
}
