//! Configuration spaces: the spaces a planner searches and a trajectory is
//! sampled in.
//!
//! A configuration is the smallest set of numbers that places an agent for
//! planning: a position in the plane, a planar pose, later a pose in 3D or a
//! set of joint angles. It is not the estimator's state. A state carries
//! velocities, biases and covariance laid out for a filter; a configuration is
//! read out of a state where a planner or follower needs one, and nothing
//! flows back.
//!
//! The base trait, [`Configuration`], holds only what every space answers the
//! same way for every planner. Anything a planner may weigh differently lives
//! beside it rather than in it:
//!
//! - **Path length** belongs to the space. It is a physical length in meters,
//!   and a trajectory's arc length is built from it.
//! - **Distance** belongs to a [`Metric`], a separate value over a space. One
//!   space has many metrics (weighted, Dubins, Reeds–Shepp), each a planner's
//!   own measure of nearness or cost.
//!
//! Abilities only some planners need, such as sampling within bounds, arrive
//! as narrower traits named in that planner's bounds, so a space never has to
//! implement what no caller asks of it.

mod r2;
mod se2;

pub use r2::{EuclideanR2Metric, R2};
pub use se2::{InvalidSe2Weight, Se2, WeightedSe2Metric};

use crate::spatial::{transforms::ConventionOf, FrameAwareState, FrameId};

use std::marker::PhantomData;

/// One point in a configuration space.
///
/// Implemented by the configuration value itself, so a space is named by its
/// type: `Se2<F>`, `R2<F>`.
pub trait Configuration: Sized {
    /// The number of coordinates a configuration has: 2 for a point in the
    /// plane, 3 for a planar pose.
    ///
    /// A method rather than a constant so that a space described at runtime,
    /// such as a joint space read from a robot description, can answer it.
    fn dimension(&self) -> usize;

    /// The physical length of the path from `self` to `other` that
    /// [`interpolate`](Self::interpolate) traces, in the space's own unit:
    /// meters for a position or a planar pose, radians for a space of
    /// revolute joints.
    ///
    /// A length, not a cost: it never depends on a planner's weights. A space
    /// whose coordinates are not all lengths says which part it measures; a
    /// planar pose measures translation only, so a turn in place has length
    /// zero.
    fn path_length(&self, other: &Self) -> f64;

    /// The configuration a `fraction` of the way from `self` to `other`.
    ///
    /// `fraction` is clamped to [0, 1]: a fraction computed from two times can
    /// land just past either end. At 0 and 1 the endpoints come back exactly,
    /// not as the result of arithmetic that may miss them by a rounding error.
    fn interpolate(&self, other: &Self, fraction: f64) -> Self;
}

/// A configuration that can be read out of an estimator's state: where the
/// agent is now, for a planner's start, a follower or a goal check.
///
/// `K` is the key that says what in the state to read, chosen by each space:
/// `()` when the state's position is the whole answer, a [`BodyFrame`] when the
/// body's orientation is needed too, a list of joints for a joint space. It is
/// a parameter of the trait, not an associated type, so a space can be read
/// with a key of every body convention.
///
/// Separate from [`Configuration`] because only these few callers read a
/// configuration from a state; a planner that never does need not name it.
pub trait FromState<K>: Configuration {
    /// The configuration `state` holds, or `None` when the state does not
    /// track a block the configuration needs (a state with no orientation has
    /// no heading).
    fn from_state(state: &FrameAwareState, key: &K) -> Option<Self>;
}

/// A measure of how far apart two configurations of `C` are, for a planner to
/// rank and search by.
///
/// Separate from [`Configuration::path_length`]: a metric may weigh a turn
/// against a meter, or measure along the shortest path a vehicle can actually
/// drive. Its units are whatever the metric documents.
pub trait Metric<C: Configuration> {
    /// The distance from `a` to `b` under this metric.
    fn distance(&self, a: &C, b: &C) -> f64;
}

/// The key for reading a configuration that needs a body's orientation: the
/// body's frame, together with the convention `B` its axes are named in.
///
/// The code that builds one, once from the agent's description, is the one
/// place that states the body's convention; every read with it then names that
/// convention to the state's typed orientation reader.
#[derive(Debug)]
pub struct BodyFrame<B: ConventionOf> {
    frame: FrameId,
    _convention: PhantomData<B>,
}

impl<B: ConventionOf> Clone for BodyFrame<B> {
    fn clone(&self) -> Self {
        Self {
            frame: self.frame.clone(),
            _convention: PhantomData,
        }
    }
}

impl<B: ConventionOf> BodyFrame<B> {
    /// Names `frame` as a body whose axes follow convention `B`.
    pub fn new(frame: FrameId) -> Self {
        Self {
            frame,
            _convention: PhantomData,
        }
    }

    /// The body's frame.
    pub fn frame(&self) -> &FrameId {
        &self.frame
    }
}
