//! Planner family: the node adapter, its bus-input assembly, its config, and
//! registration.
//!
//! - `node` — `SearchPlannerNode`, the adapter generic over any `SearchPlanner`.
//! - `input` — assembles `SearchPlannerInputs` from the bus.
//! - `config` — `AStarPlannerConfig`, the `[nodes.<name>]` section of an
//!   `AStar` entry.
//! - `register` — registers the `AStar` kind.
//!
//! Only the `register` fn crosses the family boundary; the factory builds the
//! node and declares its goal as an outside input.

mod config;
mod input;
mod node;
mod register;

pub(crate) use register::register;
