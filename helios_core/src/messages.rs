use crate::frames::FrameAwareState;
use crate::types::{Control, FrameHandle};
use nalgebra::{Isometry3, Matrix6, Point3, Vector3, Vector6};

// =========================================================================
// == Perception-Specific Data Structures ==
// =========================================================================

/// Represents a single point from a sensor like a LiDAR.
#[derive(Debug, Clone, Copy)]
pub struct Point {
    /// The 3D position of the point in the SENSOR's local coordinate frame.
    pub position: Point3<f64>,
    /// Optional: The intensity of the laser return for this point.
    pub intensity: Option<f32>,
}

/// A structured representation of a point cloud from a sensor.
#[derive(Clone, Debug)]
pub struct PointCloud {
    /// The handle of the sensor that generated this point cloud.
    pub sensor_handle: FrameHandle,
    /// The timestamp of when the scan was captured.
    pub timestamp: f64,
    /// The collection of points that make up the scan.
    pub points: Vec<Point>,
}

// =========================================================================
// == Core Message and Data Enums ==
// =========================================================================

/// A rich, self-describing container for all sensor data.
#[derive(Clone, Debug)]
pub enum MeasurementData {
    Imu6Dof(Vector6<f64>),
    Imu9Dof {
        accel_gyro: Vector6<f64>,
        mag: Vector3<f64>,
    },
    GpsPosition(Vector3<f64>),
    Magnetometer(Vector3<f64>),

    // The variant now holds our new, descriptive PointCloud struct.
    PointCloud(PointCloud),
    // You can add more variants here as needed, like `GpsPositionVelocity`.
}

// NOTE: We REMOVE the `impl MeasurementData` block with `as_primary_slice`.
// This logic was brittle and is no longer needed. The decision of how to
// convert data to a DVector now correctly lives inside each `MeasurementModel`'s
// `predict_measurement` function, which is much safer.

/// The generic message that carries all sensor data through the system.
#[derive(Clone, Debug)]
pub struct MeasurementMessage {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub timestamp: f64,
    pub data: MeasurementData,
}

/// The universal input packet for all `StateEstimator` implementations.
pub enum ModuleInput<'a> {
    TimeStep { dt: f64, current_time: f64 },
    // Control and PoseUpdate are currently not used in our final `predict`/`update`
    // design, but we can keep them for future flexibility or for other module types.
    Control { u: &'a Control },
    Measurement { message: &'a MeasurementMessage },
    PoseUpdate { pose: &'a FrameAwareState },
}

// =========================================================================
// == Public API Messages (Topic Data) ==
// =========================================================================

/// The primary output of the estimator. Represents the robot's full dynamic state.
/// This is the standardized message that planners and controllers will consume.
#[derive(Clone, Debug, Default)]
pub struct Odometry {
    pub timestamp: f64,
    pub pose: Isometry3<f64>,
    pub velocity_body: Vector6<f64>,
    pub linear_acceleration_body: Vector3<f64>,
    pub angular_acceleration_body: Vector3<f64>,
    pub pose_covariance: Matrix6<f64>,
    pub velocity_covariance: Matrix6<f64>,
}
