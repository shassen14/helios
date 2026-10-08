//! The seam passes. A seam is a channel several nodes feed and one consumer
//! reads; each pass reads its stack section, finds the named members among the
//! built nodes, and adds the node that combines them.
//!
//! - `estimate` — the agent's estimate, forwarded from one estimator, and
//!   the `odom → base_link` edge.
//! - `reference` — the guidance reference the controllers track.
//! - `command` — the named command folds the allocators read, and the
//!   built-in command types.
//! - `actuators` — the one actuator command the body applies, merged from the
//!   members' partials and checked against the body.
//! - `members` — resolving a section's members to the channels they write.

pub(super) mod actuators;
pub(super) mod command;
pub(super) mod estimate;
mod members;
pub(super) mod reference;
