//! The runtime glue that turns transform-edge bus traffic into a queryable tree.
//!
//! [`TfBuffer`] lives in `helios_core` and is deliberately clock-free and
//! bus-free — it knows how to compose and interpolate edges, nothing about where
//! samples come from. [`TfService`] is the missing half: each tick it drains the
//! per-edge channels (the vocabulary in [`crate::channels::tf`]) into that buffer,
//! then hands the buffer out as a read-only [`TfProvider`] for consumers to query.
//! The drain is driven by an *explicit* list of edge keys, not a "read every
//! transform" scan — the bus has no such primitive, and each edge is its own
//! last-known-good slot.
//!
//! - `config` — [`TfBufferConfig`], the stack's `[tf]` section: how much
//!   history the buffer keeps.
//! - `service` — [`TfService`] and [`DrainedEdge`].
//!
//! [`TfBuffer`]: helios_core::spatial::transforms::tf::buffer::TfBuffer
//! [`TfProvider`]: helios_core::prelude::TfProvider

mod config;
mod service;

pub use self::config::TfBufferConfig;
pub use self::service::{DrainedEdge, TfService};
