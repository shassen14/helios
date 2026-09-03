//! Controller family: the node adapter, its bus-input assembly, and registration.
//!
//! - `node` — `ControllerNode<C>`, the generic pipeline adapter.
//! - `input` — assembles `ControlInputs` from the bus.
//! - `register` — registers the built-in controller factories.
//!
//! Only the `register` fn crosses the family boundary; the node and input types
//! are wired together internally and boxed as `Box<dyn PipelineNode>` by the
//! factory.

mod input;
mod node;
mod register;

pub(crate) use register::register;
