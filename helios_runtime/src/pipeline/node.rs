use crate::observe::buffer::NodeObservations;
use crate::observe::observation::ObservedValue;
use crate::port::{PortBus, PortDescriptor};

use helios_core::prelude::TfProvider;
use helios_core::spatial::primitives::MonotonicTime;

use std::sync::Arc;

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
    fn execute(&self, bus: &PortBus, tf: &dyn TfProvider, tick: TickContext);
}

/// Per-execution context passed to every [`PipelineNode::execute`] call.
///
/// `now` and `dt` are supplied by the host to [`AutonomyPipeline::tick`], so
/// simulation and hardware share the same clock semantics. `node_id` lets a node
/// tag the values it writes to the bus with its own identity for diagnostics.
///
/// The context also carries the node's outlet for watchers:
/// [`emit`](Self::emit) and [`emit_with`](Self::emit_with) record values that
/// tests, visualizers, and recorders read after the tick. Nothing in the
/// pipeline reads them back, and emitting returns nothing to branch on, so
/// what is watched never changes what the pipeline computes. The context
/// borrows its node's buffer for one tick, so it can't be kept past it.
pub struct TickContext<'a> {
    pub now: MonotonicTime,
    pub dt: f64,
    pub node_id: NodeId,
    /// The node's own buffer, or `None` for a detached context.
    observations: Option<&'a NodeObservations>,
}

impl<'a> TickContext<'a> {
    /// A context whose emits go to `observations`, the running node's own
    /// buffer.
    pub(crate) fn new(
        now: MonotonicTime,
        dt: f64,
        node_id: NodeId,
        observations: &'a NodeObservations,
    ) -> Self {
        Self {
            now,
            dt,
            node_id,
            observations: Some(observations),
        }
    }

    /// A context outside any pipeline, whose emits go nowhere. Used to run a
    /// node's [`execute`](PipelineNode::execute) directly, as unit tests do.
    pub fn detached(now: MonotonicTime, dt: f64, node_id: NodeId) -> TickContext<'static> {
        TickContext {
            now,
            dt,
            node_id,
            observations: None,
        }
    }

    /// Records `value` under `leaf` if a watcher asked for it, else does
    /// nothing.
    ///
    /// `leaf` is the name the node declares for the value, e.g.
    /// `aiding.gps.nis`. `timestamp` is the time the value describes, which
    /// is not always `now`: a measurement update stamps the reading's time.
    /// For a value that is costly to compute, use [`emit_with`](Self::emit_with).
    ///
    /// In debug builds, emitting a leaf the node didn't declare panics, naming
    /// the node and the leaf.
    pub fn emit(&self, leaf: &str, timestamp: MonotonicTime, value: impl Into<ObservedValue>) {
        let Some((buffer, leaf)) = self.watched(leaf) else {
            return;
        };

        buffer.record(leaf, timestamp, value.into());
    }

    /// Like [`emit`](Self::emit), but `make_value` runs only when `leaf` is
    /// watched, so an unwatched value costs one check.
    ///
    /// `make_value` must not change the node's state; it may not run at all.
    pub fn emit_with<V>(&self, leaf: &str, timestamp: MonotonicTime, make_value: impl FnOnce() -> V)
    where
        V: Into<ObservedValue>,
    {
        let Some((buffer, leaf)) = self.watched(leaf) else {
            return;
        };

        buffer.record(leaf, timestamp, make_value().into());
    }

    /// The buffer and stored leaf name to record into, if this context has a
    /// buffer and `leaf` is watched in it.
    ///
    /// Checks that `leaf` is declared before asking whether it is watched, so
    /// an undeclared emit is caught even when nothing on the node is watched.
    fn watched(&self, leaf: &str) -> Option<(&'a NodeObservations, Arc<str>)> {
        let buffer = self.observations?;
        buffer.assert_declared(leaf);
        let leaf = buffer.watched_leaf(leaf)?;
        Some((buffer, leaf))
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observe::observation::Observation;

    use std::cell::Cell;

    const NODE: &str = "estimator";
    const NODE_ID: NodeId = 0;
    const NIS: &str = "aiding.gps.nis";
    const UNDECLARED: &str = "aiding.gps.nsi";
    const DT: f64 = 0.01;

    fn buffer(watched: bool) -> NodeObservations {
        let mut buffer = NodeObservations::new(NODE, [NIS]);
        buffer.set_watched(NIS, watched);
        buffer
    }

    fn context(buffer: &NodeObservations) -> TickContext<'_> {
        TickContext::new(MonotonicTime(2.0), DT, NODE_ID, buffer)
    }

    fn drain(buffer: &NodeObservations) -> Vec<Observation> {
        let mut out = Vec::new();
        buffer.drain_into(&mut out);
        out
    }

    fn observation(t: f64, value: f64) -> Observation {
        Observation {
            node: NODE.into(),
            leaf: NIS.into(),
            timestamp: MonotonicTime(t),
            value: ObservedValue::Scalar(value),
        }
    }

    #[test]
    fn watched_emits_record_with_their_own_timestamp() {
        let buffer = buffer(true);
        let tick = context(&buffer);

        tick.emit(NIS, MonotonicTime(1.5), 0.5);
        tick.emit_with(NIS, MonotonicTime(2.0), || 1.5);

        assert_eq!(
            drain(&buffer),
            vec![observation(1.5, 0.5), observation(2.0, 1.5)]
        );
    }

    #[test]
    fn unwatched_lazy_emit_never_builds_the_value() {
        let buffer = buffer(false);
        let tick = context(&buffer);
        let ran = Cell::new(false);

        tick.emit_with(NIS, tick.now, || {
            ran.set(true);
            0.5
        });

        assert!(!ran.get());
        assert!(drain(&buffer).is_empty());
    }

    // `expected` takes only a literal, so it repeats `UNDECLARED`. The check
    // is compiled out of release builds, and so is this test.
    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "node \"estimator\" emitted undeclared leaf \"aiding.gps.nsi\"")]
    fn undeclared_emit_panics_even_when_nothing_is_watched() {
        let buffer = buffer(false);
        let tick = context(&buffer);

        tick.emit(UNDECLARED, tick.now, 0.5);
    }

    #[test]
    fn detached_context_emits_go_nowhere() {
        let tick = TickContext::detached(MonotonicTime(2.0), DT, NODE_ID);
        let ran = Cell::new(false);

        tick.emit(NIS, tick.now, 0.5);
        tick.emit_with(NIS, tick.now, || {
            ran.set(true);
            0.5
        });

        assert!(!ran.get());
    }
}
