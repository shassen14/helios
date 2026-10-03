//! Typed bus channels, the blackboard they live on, and the declarations that
//! wire nodes to it.
//!
//! Three concerns, one per submodule:
//!
//! - [`channel`] — channel *identity*: the kind partition, the per-kind
//!   constructors ([`SensorChannel`], [`InternalChannel`], [`OracleChannel`],
//!   [`HealthChannel`]), and [`ChannelKey`].
//! - [`bus`] — the [`PortBus`] blackboard: one last-known-good slot per
//!   channel, each with a write [`SlotVersion`].
//! - [`descriptor`] — the [`PortDescriptor`] that declares what each node reads
//!   from and writes to the bus, and the kind-fenced builders that construct it.
//!
//! All three are re-exported here, so callers write `crate::port::{ChannelKey,
//! PortBus, …}` without tracking which file a type lives in.

pub mod bus;
pub mod channel;
pub mod descriptor;

pub use bus::*;
pub use channel::*;
pub use descriptor::*;
