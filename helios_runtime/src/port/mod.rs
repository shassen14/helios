//! Typed bus channels and the blackboard they live on.
//!
//! Two concerns, one per submodule:
//!
//! - [`channel`] — channel *identity*: the kind partition, the per-kind
//!   constructors ([`SensorChannel`], [`InternalChannel`], [`OracleChannel`],
//!   [`HealthChannel`]), and [`ChannelKey`].
//! - [`bus`] — the [`PortBus`] blackboard and the [`PortDescriptor`] that
//!   declares what each node reads from and writes to it.
//!
//! Both are re-exported here, so callers write `crate::port::{ChannelKey,
//! PortBus, …}` without tracking which file a type lives in.

pub mod bus;
pub mod channel;

pub use bus::*;
pub use channel::*;
