use crate::prelude::MonotonicTime;
use crate::spatial::conventions::Frame;
use crate::spatial::quantities::FreeVector;
use crate::spatial::FrameId;

use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};
use serde::Serialize;

/// The velocity of one frame relative to another, expressed in the axes of a
/// third, as it was at `stamp`.
///
/// A velocity names three frames, and each answers a different question:
/// `moving` is the frame whose origin and axes move, `relative_to` is the
/// frame they move against, and `expressed_in` is the frame whose axes the
/// numbers use. A body's velocity relative to the world, expressed in the
/// body, is the forward speed a wheel odometer reads; the same velocity
/// expressed in the world is the ground track a GNSS receiver reads. The
/// message states all three rather than leaving any implied, so a reader
/// never re-expresses a twist it assumed was in other axes.
///
/// `linear` is the velocity of `moving`'s origin and `angular` the angular
/// velocity of `moving`'s axes, both in `F`, the convention of
/// `expressed_in`. Re-expressing in another frame is then a pure rotation of
/// both, as the reference point does not change.
#[derive(Debug)]
pub struct StampedTwist<F: Frame> {
    /// The frame whose motion this is.
    pub moving: FrameId,
    /// The frame the motion is measured against.
    pub relative_to: FrameId,
    /// The frame whose axes `linear` and `angular` use.
    pub expressed_in: FrameId,
    /// When this velocity held.
    pub stamp: MonotonicTime,
    /// The velocity of `moving`'s origin (m/s).
    pub linear: FreeVector<F>,
    /// The angular velocity of `moving`'s axes (rad/s).
    pub angular: FreeVector<F>,
}

// Hand-written for the same reason as `FreeVector`'s: a derived `Clone`
// would constrain `F: Clone`, which generic callers over `F: Frame` cannot
// satisfy.
impl<F: Frame> Clone for StampedTwist<F> {
    fn clone(&self) -> Self {
        Self {
            moving: self.moving.clone(),
            relative_to: self.relative_to.clone(),
            expressed_in: self.expressed_in.clone(),
            stamp: self.stamp,
            linear: self.linear,
            angular: self.angular,
        }
    }
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
