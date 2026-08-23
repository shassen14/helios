//! Control references — the per-rung target a controller tracks.
//!
//! [`ControlReference`] is the marker trait; each concrete reference is a typed,
//! data-only carrier of what a controller should follow (a body twist today; an
//! acceleration- or wrench-carrying reference, or a trajectory for MPC, later).
//! The reference is *not* a control role — feedforward is a summed node, not a
//! field here.

pub mod body_twist;

pub use body_twist::BodyTwistRef;

use std::any::Any;

pub trait ControlReference: Any + Send + Sync {}
