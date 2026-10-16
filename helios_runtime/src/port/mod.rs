//! Typed bus channels, the blackboard they live on, and the declarations that
//! wire nodes to it.
//!
//! Six concerns, one per submodule:
//!
//! - [`kind`] — the [`ChannelKind`] partition and the table of which
//!   [`NodeClass`] may read and write each kind.
//! - [`channel`] — channel *identity*: [`ChannelKey`], one kind, payload type
//!   and name.
//! - [`keyed`] — [`Keyed`] payloads, whose channels hold one value per
//!   [`InstanceKey`].
//! - [`bus`] — the [`PortBus`] blackboard: one last-known-good slot per
//!   channel, or per instance of a keyed channel, each with a write
//!   [`SlotVersion`].
//! - [`descriptor`] — the [`PortDescriptor`] that declares what each node reads
//!   from and writes to the bus, and the [`PortDescriptorBuilder`] that
//!   constructs it.
//! - [`description`] — the [`ChannelDescription`] of every channel the pipeline
//!   sees, with the [`Writer`] that writes it: the body, a node or the outside.
//!
//! All six are re-exported here, so callers write `crate::port::{ChannelKey,
//! PortBus, …}` without tracking which file a type lives in.

pub mod bus;
pub mod channel;
pub mod description;
pub mod descriptor;
pub mod keyed;
pub mod kind;

pub use bus::*;
pub use channel::*;
pub use description::*;
pub use descriptor::*;
pub use keyed::*;
pub use kind::*;
