use crate::port::{PortBus, PortDescriptor};
use crate::runtime::AgentRuntime;

use helios_core::data::primitives::MonotonicTime;

pub trait PipelineNode: Send + Sync {
    fn name(&self) -> &str;

    fn port_descriptor(&self) -> &PortDescriptor;

    fn execute(&self, bus: &PortBus, runtime: &dyn AgentRuntime, tick: TickContext);
}

pub struct TickContext {
    pub now: MonotonicTime,
    pub dt: f64,
    pub node_id: NodeId,
}

pub type NodeId = u32;
