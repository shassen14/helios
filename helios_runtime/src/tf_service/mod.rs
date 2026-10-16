//! The runtime glue that turns transform-edge bus traffic into a queryable tree.
//!
//! [`TfBuffer`] lives in `helios_core` and is deliberately clock-free and
//! bus-free — it knows how to compose and interpolate edges, nothing about where
//! samples come from. [`TfService`] is the missing half: each tick it drains the
//! keyed `tf` channel ([`crate::channels::tf`]), one instance per edge, into that
//! buffer, then hands the buffer out as a read-only [`TfProvider`] for consumers
//! to query.
//!
//! - `config` — [`TfBufferConfig`], the stack's `[tf]` section: how much
//!   history the buffer keeps.
//! - `service` — [`TfService`]; [`SeedRejected`] and
//!   [`RefusedMounts`] ([`MisreadMount`]), the two ways a host's static feed
//!   fails a pipeline.
//!
//! [`TfBuffer`]: helios_core::spatial::transforms::tf::buffer::TfBuffer
//! [`TfProvider`]: helios_core::prelude::TfProvider

mod config;
mod service;

pub use self::config::TfBufferConfig;
pub use self::service::{MisreadMount, RefusedMounts, SeedRejected, TfService};
