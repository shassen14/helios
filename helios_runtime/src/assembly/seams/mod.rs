//! The seam passes. A seam is a channel several nodes feed and one consumer
//! reads; each pass reads its stack section, finds the named members among the
//! built nodes, and adds the node that combines them.
//!
//! - `reference` — the guidance reference the controllers track.
//! - `command` — the named command folds the allocators read, and the
//!   built-in command types.
//! - `members` — resolving a section's members to the channels they write.

pub(super) mod command;
mod members;
pub(super) mod reference;
