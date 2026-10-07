//! Path-follower family: the node adapter, its bus-input assembly, its config
//! sections, and registration.
//!
//! - `node` — `PathFollowerNode`, the adapter generic over any `PathFollower`.
//! - `input` — assembles `PathFollowerInputs` from the bus.
//! - `config` — the `[nodes.<name>]` sections of the `PurePursuit` and
//!   `SteeringPid` kinds.
//! - `register` — registers those kinds.
//!
//! Only the `register` fn crosses the family boundary; the node and input
//! types are wired together internally and boxed as `Box<dyn PipelineNode>` by
//! the factory.

mod config;
mod input;
mod node;
mod register;

pub(crate) use register::register;
