use crate::port::{PortBus, PortDescriptor};
use crate::runtime::AgentRuntime;

use helios_core::data::primitives::MonotonicTime;

/// One unit of computation in an [`AutonomyPipeline`].
///
/// Nodes declare their bus interaction (inputs, outputs, rate) via
/// [`port_descriptor`](PipelineNode::port_descriptor). The pipeline calls
/// [`execute`](PipelineNode::execute) once per tick on every node whose
/// [`RateTimer`](super::rate_gate::RateTimer) is due — in topological order
/// across levels.
///
/// Implementations must be `Send + Sync`. `execute` takes `&self`; any mutable
/// algorithm state (e.g. an EKF filter) must live behind a `Mutex` or atomic.
/// This is what lets a level later be executed in parallel without changing
/// the trait signature.
///
/// All reads and writes go through `bus`. Producers stamp values with
/// `tick.now` and `tick.node_id`; consumers should early-return when an
/// input is `None` (cold-start, sensor dropout, or a rate-gated upstream
/// node that has not fired yet).
pub trait PipelineNode: Send + Sync {
    /// Human-readable identifier used in build errors and diagnostics.
    fn name(&self) -> &str;

    /// Describes the bus channels this node reads and writes, plus its
    /// execution rate. Returned by reference because the descriptor is
    /// immutable after construction.
    fn port_descriptor(&self) -> &PortDescriptor;

    /// Run one iteration. Called by [`AutonomyPipeline::tick`](super::AutonomyPipeline::tick).
    fn execute(&self, bus: &PortBus, runtime: &dyn AgentRuntime, tick: TickContext);
}

/// Per-execution context passed to every [`PipelineNode::execute`] call.
///
/// `now` and `dt` are sourced from the [`AgentRuntime`] so simulation and
/// hardware share the same clock semantics. `node_id` lets a node tag the
/// values it writes to the bus with its own identity for diagnostics.
pub struct TickContext {
    pub now: MonotonicTime,
    pub dt: f64,
    pub node_id: NodeId,
}

/// Build-time-assigned identifier for a [`PipelineNode`].
///
/// IDs are assigned in level-major order starting at `0` and index into the
/// pipeline's rate-timer array. Used as the `producer` field on
/// [`Stamped`](crate::stamped::Stamped) values written to the bus.
pub type NodeId = u32;

/// Sentinel [`NodeId`] used as the `producer` field on bus writes that
/// originate **outside** the pipeline graph — e.g. a mission goal injected
/// by a Zenoh bridge or sensor batches written by the host tick system.
///
/// `NodeId::MAX` is chosen so the sentinel cannot collide with a real
/// assigned ID (which counts up from 0 and is bounded by the node count).
pub const HOST_PRODUCER_ID: NodeId = NodeId::MAX;
