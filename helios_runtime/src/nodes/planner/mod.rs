//! Planner family: the node adapter, its bus-input assembly, and registration.
//!
//! - `node` — `SearchPlannerNode`, the adapter generic over any `SearchPlanner`.
//! - `input` — assembles `SearchPlannerInputs` from the bus.
//! - `register` — registers the built-in search-planner factories.
//!
//! Only the `register` fn crosses the family boundary; the node and input types
//! are wired together internally and boxed as `Box<dyn PipelineNode>` by the
//! factory.

mod input;
mod node;
mod register;

pub(crate) use register::register;
