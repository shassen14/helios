//! Path-follower family: the node adapter, its bus-input assembly, and registration.
//!
//! - `node` — `PathFollowerNode`, the adapter generic over any `PathFollower`.
//! - `input` — assembles `PathFollowerInputs` from the bus.
//! - `register` — registers the built-in path-follower factories.
//!
//! Only the `register` fn crosses the family boundary; the node and input types
//! are wired together internally and boxed as `Box<dyn PipelineNode>` by the
//! factory.

mod input;
mod node;
mod register;

pub(crate) use register::register;
