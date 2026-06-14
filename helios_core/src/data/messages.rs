use crate::frames::RobotState;

use nalgebra::{DVector, Isometry3, Matrix6, Vector3, Vector6};
use serde::Serialize;

// todo: should I change to linear_velocity and angular_velocity
// names? I have some inconsistency with naming.
#[derive(Clone, Debug, Default, Serialize)]
pub struct Twist {
    pub linear: Vector3<f64>,
    pub angular: Vector3<f64>,
}

/// The primary output of the estimator. Represents the robot's full dynamic state.
#[derive(Clone, Debug, Default, Serialize)]
pub struct Odometry {
    pub(crate) timestamp: f64,
    pub(crate) pose: Isometry3<f64>,
    pub(crate) velocity_body: Vector6<f64>,
    pub(crate) linear_acceleration_body: Vector3<f64>,
    pub(crate) angular_acceleration_body: Vector3<f64>,
    pub(crate) pose_covariance: Matrix6<f64>,
    pub(crate) velocity_covariance: Matrix6<f64>,
}

/// Reference trajectory point for the current time step.
///
/// Provided by the planner; consumed by path follower and controller.
/// `state_dot` is `None` when the planner cannot or does not provide it
/// (e.g. a waypoint planner). Feedforward controllers degrade gracefully
/// to pure feedback and log a diagnostic when `state_dot` is absent.
#[derive(Clone)]
pub struct TrajectoryPoint {
    pub state: RobotState,
    /// x_dot_ref = f(x_ref, u_nominal, t). None = feedforward unavailable.
    pub state_dot: Option<DVector<f64>>,
    pub time: f64,
}
