//! Deproject node: flattens a host-published range field into a point cloud.
//!
//! - `node` — `DeprojectNode`, the range-field-to-point-cloud conversion.
//! - `config` — `DeprojectConfig`, the `[nodes.<name>]` section.
//! - `register` — registers the `Deproject` kind and its build function.

mod config;
mod node;
mod register;

pub(crate) use node::DeprojectNode;
pub(crate) use register::register;
