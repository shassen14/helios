//! The seam passes. A seam is a channel several nodes feed and one consumer
//! reads; each pass reads its stack section, finds the named members among the
//! built nodes, and adds the node that combines them.
//!
//! - `reference` — the guidance reference the controllers track.

pub(super) mod reference;
