//! Typed bus channels, the blackboard they live on, and the declarations that
//! wire nodes to it.
//!
//! Four concerns, one per submodule:
//!
//! - [`channel`] — channel *identity*: the kind partition, the per-kind
//!   constructors ([`SensorChannel`], [`InternalChannel`], [`OracleChannel`],
//!   [`HealthChannel`]), and [`ChannelKey`].
//! - [`bus`] — the [`PortBus`] blackboard: one last-known-good slot per
//!   channel, each with a write [`SlotVersion`].
//! - [`descriptor`] — the [`PortDescriptor`] that declares what each node reads
//!   from and writes to the bus, and the kind-fenced builders that construct it.
//! - [`description`] — the [`ChannelDescription`] of every channel the pipeline
//!   sees, with the [`Writer`] that writes it: the body, a node or the outside.
//!
//! All four are re-exported here, so callers write `crate::port::{ChannelKey,
//! PortBus, …}` without tracking which file a type lives in.

pub mod bus;
pub mod channel;
pub mod description;
pub mod descriptor;

pub use bus::*;
pub use channel::*;
pub use description::*;
pub use descriptor::*;
