//! Command-space vocabulary: the typed outputs a `Controller` can produce.
//!
//! Each command is a standalone struct generic over its coordinate frame.
//! [`Wrench`] is the first; sibling commands (rate/thrust, body velocity, …)
//! follow the same shape and land here beside it.

pub mod twist;
pub mod wrench;

pub use twist::Twist;
pub use wrench::Wrench;

use crate::frames::conventions::Flu;

/// A [`Wrench`] in the body [`Flu`](crate::frames::conventions::Flu) frame — the
/// form actuator terminals consume.
pub type BodyWrench = Wrench<Flu>;

pub type BodyTwist = Twist<Flu>;
