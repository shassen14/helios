use std::any::{Any, TypeId};
use std::collections::HashMap;
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

pub struct PortBus {
    slots: HashMap<TypeId, Option<Box<dyn Any + Send>>>,
}

impl PortBus {
    pub fn new(all_type_ids: Vec<TypeId>) -> Self {
        let slots = all_type_ids.into_iter().map(|id| (id, None)).collect();
        Self { slots }
    }

    pub fn write<T: Any + Send>(&mut self, value: T) {
        todo!();
    }

    pub fn read<T: Any>(&self) -> Option<&T> {
        self.slots.get(&TypeId::of::<T>())?.as_ref()?.downcast_ref()
    }
}
