use std::any::TypeId;
use std::option::Option;

/// Defines what a node needs and produces
///
/// Port Identity is `TypeId` because each port type is a Rust Type.
pub struct PortDescriptor {
    /// Types that must be on the bus before this node runs
    pub required_inputs: Vec<TypeId>,

    /// Types that node uses if present
    pub optional_inputs: Vec<TypeId>,

    /// Types this node writes to the bus when it executes
    pub outputs: Vec<TypeId>,

    /// Execution rate in Hz. `None` means every tick.
    pub rate: Option<f64>,
}
