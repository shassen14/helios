//! Path-follower family: the node adapter, its bus-input assembly, and registration.
//!
//! - `node` — `PathFollowerNode`, the adapter generic over any `PathFollower`.
//! - `input` — assembles `PathFollowerInputs` from the bus.
//! - `assemble` — resolves the planner output channel the follower reads from,
//!   invoked by the assembler when wiring the follower.
//! - `register` — registers the built-in path-follower factories.
//!
//! The `register` fn and `resolve_path_channel` cross the family boundary; the
//! node and input types are wired together internally and boxed as
//! `Box<dyn PipelineNode>` by the factory.

mod assemble;
mod input;
mod node;
mod register;

pub(crate) use assemble::resolve_path_channel;
pub(crate) use register::register;
