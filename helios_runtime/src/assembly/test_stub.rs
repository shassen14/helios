//! A do-nothing node for assembly tests, where only a node's name and outputs
//! matter.

use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{AlgorithmNodePortDescriptor, InternalChannel, PortBus, PortDescriptor};

use helios_core::control::actuators::{ActuatorCommand, ActuatorDrive};
use helios_core::prelude::TfProvider;

/// A node that does nothing when it runs.
struct StubNode {
    name: String,
    descriptor: PortDescriptor,
}

impl PipelineNode for StubNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
}

/// A node named `name` writing `outputs`.
pub(crate) fn stub(name: &str, outputs: Vec<InternalChannel>) -> Box<dyn PipelineNode> {
    let descriptor = outputs
        .into_iter()
        .fold(AlgorithmNodePortDescriptor::new(), |builder, output| {
            builder.output_internal(output)
        })
        .build();
    Box::new(StubNode {
        name: name.to_string(),
        descriptor,
    })
}

/// A node named `name` writing one `ActuatorCommand` on a channel named after
/// itself, as every allocator does, declaring that it drives `drives`.
pub(crate) fn stub_driving(name: &str, drives: Vec<ActuatorDrive>) -> Box<dyn PipelineNode> {
    let descriptor = AlgorithmNodePortDescriptor::new()
        .output_actuator_command(InternalChannel::named::<ActuatorCommand>(name), drives)
        .build();
    Box::new(StubNode {
        name: name.to_string(),
        descriptor,
    })
}
