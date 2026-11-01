//! The kinds of trajectory key.

mod geometry;
mod joint;

pub use geometry::{GeometryId, GeometryKey, PositionDim};
pub use joint::{Joint, JointGroup, JointGroupError, JointType};
