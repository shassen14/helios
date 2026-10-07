//! Allocator family: the node adapter, the kinds' config sections, and
//! registration.
//!
//! - `node` — `AllocatorNode<A>`, the generic pipeline adapter.
//! - `config` — the `[nodes.<name>]` section of each allocator kind.
//! - `register` — registers the allocator kinds.

mod config;
mod node;
mod register;

pub(crate) use register::register;
