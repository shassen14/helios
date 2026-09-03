//! Command-space vocabulary: the typed outputs a `Controller` can produce.
//!
//! Commands come in two shapes. **Spatial** commands ([`Wrench`], [`Twist`]) are
//! generic over a coordinate frame — a 6-DOF vector whose basis `F` says how to
//! rotate it. **Per-degree-of-freedom** commands ([`DriveForce`], [`SteerAngle`])
//! are scalar and axis-less: a single number has no basis to rotate, so it carries
//! no frame, and the actuator identity is stamped downstream at allocation. Every
//! command is a standalone struct following one of these two shapes; siblings land
//! here beside them.

pub mod drive_force;
pub mod steer_angle;
pub mod twist;
pub mod twist_intent;
pub mod wrench;

pub use drive_force::DriveForce;
pub use steer_angle::SteerAngle;
pub use twist::Twist;
pub use twist_intent::TwistIntent;
pub use wrench::Wrench;

use crate::frames::conventions::Flu;

/// A [`Wrench`] in the body [`Flu`](crate::frames::conventions::Flu) frame — the
/// form actuator terminals consume.
pub type BodyWrench = Wrench<Flu>;

pub type BodyTwist = Twist<Flu>;
