//! Deproject node: flattens a host-published range field into a point cloud.
//!
//! - `node` — `DeprojectNode`, the range-field-to-point-cloud conversion.
//!
//! Runtime-native plumbing, not a registered family: it wraps no swappable
//! core algorithm, so the assembler constructs it directly from its config
//! variant.

mod node;

pub(crate) use node::DeprojectNode;
