//! What a node declares it reads from and writes to the bus.
//!
//! - [`declaration`] — the [`PortDescriptor`] itself, its per-input
//!   [`InputPort`] records ([`InputNeed`], [`InputTiming`]), the
//!   [`MountReading`] records of the inputs whose mounts it resolves, per-output
//!   [`OutputPort`] records, and the
//!   [`Observable`] records of what a node can emit for watchers
//!   ([`Determinism`]).
//! - [`builder`] — [`PortDescriptorBuilder`], the only way to build a
//!   descriptor from outside this crate.

pub mod builder;
pub mod declaration;

pub use builder::PortDescriptorBuilder;
pub use declaration::{
    Determinism, InputNeed, InputPort, InputTiming, MountReading, MountRule, Observable,
    OutputPort, PortDescriptor,
};
