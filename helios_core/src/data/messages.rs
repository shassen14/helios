use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};
use serde::Serialize;

/// The primary output of the estimator. Represents the robot's full dynamic state.
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
