//! The keys a trajectory's columns are declared by. A trajectory is a table:
//! one row per sample, one column per stored number, and a key is the header
//! of one or more columns, saying what those numbers mean.
//!
//! A key names a role, never a subject (ego, a track, a candidate), so one
//! set of keys serves every member of a set of trajectories.
//!
//! - [`GeometryKey`]: where, from a closed list of roles. The role decides
//!   the [`GeometryKind`](crate::kernel::manifold::geometry::GeometryKind) and
//!   the units, so neither is declared and neither can be wrong.
//! - [`Anchor`] and [`Root`]: the point a spatial key describes, and what it
//!   is measured from.
//! - [`KeyName`]: the one name grammar for joints, joint groups, and the
//!   other named keys.
//!
//! Keys stand on their own: none knows a column offset or the table it sits
//! in, so a reader that is not a table (a goal, a constraint) names the same
//! keys.

mod anchor;
mod key;
mod name;

pub use anchor::{Anchor, Root};
pub use key::{
    GeometryId, GeometryKey, Joint, JointGroup, JointGroupError, JointType, PositionDim,
};
pub use name::{KeyName, MalformedKeyName};
