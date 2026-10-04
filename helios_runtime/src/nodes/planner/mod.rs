//! Planner family: the node adapter, its bus-input assembly, and registration.
//!
//! - `node` — `SearchPlannerNode`, the adapter generic over any `SearchPlanner`.
//! - `input` — assembles `SearchPlannerInputs` from the bus.
//! - `register` — registers the built-in search-planner factories.
//!
//! The `register` fn crosses the family boundary, and so does the input
//! builder, so the assembler can build the goal key the planner reads. The node
//! and input types are otherwise wired together internally and boxed as
//! `Box<dyn PipelineNode>` by the factory.

mod input;
mod node;
mod register;

pub(crate) use input::DefaultSearchPlannerInputBuilder;
pub(crate) use register::register;
