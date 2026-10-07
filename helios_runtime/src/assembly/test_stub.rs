//! A do-nothing node for assembly tests, where only a node's name and outputs
//! matter.

use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{AlgorithmNodePortDescriptor, InternalChannel, PortBus, PortDescriptor};

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
