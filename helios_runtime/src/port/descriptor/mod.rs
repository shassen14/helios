//! What a node declares it reads from and writes to the bus.
//!
//! - [`declaration`] — the [`PortDescriptor`] itself and its per-input
//!   [`InputPort`] records ([`InputNeed`], [`InputTiming`]).
//! - [`builders`] — [`AlgorithmNodePortDescriptor`] and
//!   [`MockNodePortDescriptor`], the kind-fenced constructors that are the only
//!   way to build a descriptor from outside this crate.

pub mod builders;
pub mod declaration;

pub use builders::{AlgorithmNodePortDescriptor, MockNodePortDescriptor};
pub use declaration::{InputNeed, InputPort, InputTiming, PortDescriptor};
