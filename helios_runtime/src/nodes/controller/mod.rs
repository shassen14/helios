//! Controller family: the node adapter, its bus-input assembly, the kinds'
//! config sections, and registration.
//!
//! - `node` — `ControllerNode<C>`, the generic pipeline adapter.
//! - `input` — assembles `ControlInputs` from the bus.
//! - `config` — the `[nodes.<name>]` section of each controller kind.
//! - `register` — registers the controller kinds.
//!
//! Only the `register` fn crosses the family boundary; the node and input types
//! are wired together internally and boxed as `Box<dyn PipelineNode>` by the
//! factory.

mod config;
mod input;
mod node;
mod register;

pub(crate) use register::register;
