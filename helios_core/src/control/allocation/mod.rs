use crate::control::actuators::ActuatorCommand;

pub trait Allocator {
    type In;
    type Inputs;

    fn allocate(&mut self, command: &Self::In, inputs: &Self::Inputs) -> ActuatorCommand;
}
