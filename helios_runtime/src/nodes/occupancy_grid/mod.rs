//! Occupancy-grid family: the mapper node and its registration.
//!
//! - `node` — `OccupancyGridNode`, the `Mapper`-backed grid node.
//! - `register` — registers the built-in mapper factories.
//!
//! No input-builder submodule: the node reads its scan channel directly. Only
//! the `register` fn crosses the family boundary; the factory boxes the node as
//! `Box<dyn PipelineNode>`.

mod node;
mod register;

pub(crate) use register::register;
