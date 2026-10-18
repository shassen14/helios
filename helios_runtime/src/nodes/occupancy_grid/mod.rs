//! Occupancy-grid family: the mapper node and its registration.
//!
//! - `node` — `OccupancyGridNode`, the `Mapper`-backed grid node.
//! - `config` — `OccupancyGridConfig`, the `[nodes.<name>]` section.
//! - `register` — registers the `OccupancyGrid2D` kind and its build function.
//!
//! No input-builder submodule: the node reads its scan channel directly. Only
//! the `register` fn crosses the family boundary.

mod config;
mod node;
mod register;

pub(crate) use register::register;
