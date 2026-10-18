//! `pose_error`: how far one frame's estimated pose is from its true pose.
//! At each estimate, the truth interpolated to its stamp, and the estimate
//! minus the truth, offered as a distance, per axis, along and across the
//! body, and as a rotation and a heading. Any frame with a true pose can be
//! graded: a body's `base_link`, a sensor's mount.

pub mod config;
mod evaluator;
mod kind;

pub use kind::PoseError;
