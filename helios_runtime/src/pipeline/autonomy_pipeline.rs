//! [`AutonomyPipeline`]: a built graph of nodes over a bus, run one tick at a
//! time. Only the builder in [`build`](super::build) constructs one.

use crate::{
    channels::{control, estimate::estimate, tf::is_tf_edge},
    observe::buffer::NodeObservations,
    pipeline::rate_gate::RateTimer,
    port::{ChannelKey, Determinism, Observable, PortBus},
    prelude::{PipelineNode, Stamped, TickContext},
    NodeId,
};

use helios_core::{
    control::actuators::ActuatorCommand,
    prelude::{MonotonicTime, TfProvider},
    spatial::FrameAwareState,
};

use std::sync::Arc;
use tracing::{debug_span, trace_span};

/// The leaf under which the pipeline reports how long one run of a node took,
/// in seconds. Every node has it; nodes never declare it themselves. Wall-clock
/// time, so it differs between runs.
pub const TICK_DURATION_LEAF: &str = "tick.duration";

/// A built, validated autonomy pipeline.
///
/// Constructed only via [`PipelineBuilder::build`](crate::PipelineBuilder::build). After construction
/// the topology is fixed; the only mutable state is the bus contents, the
/// per-node timer counters, and the per-node observation buffers, all of
/// which use interior mutability so [`tick`](Self::tick) can take `&self`.
pub struct AutonomyPipeline {
    /// Nodes grouped by topological level, in execution order, each with
    /// the state the pipeline keeps for it. Walking levels in order visits
    /// nodes in [`NodeId`] order.
    pub(super) levels: Vec<Vec<ScheduledNode>>,
    /// Typed blackboard used for all intra-pipeline data exchange. Also
    /// the only way values from outside the graph enter it: the host writes
    /// body measurements and operator or mission inputs here.
    pub(super) bus: PortBus,
    /// Inputs declared as sent from outside the robot, kept from the build.
    pub(super) outside_inputs: Vec<ChannelKey>,
}

impl AutonomyPipeline {
    /// Returns a reference to the [`PortBus`] for direct read/write access.
    ///
    /// External callers use this to:
    /// - Write body measurements as they arrive, via [`PortBus::write`].
    ///   Slots keep the last written value; nothing is cleared per tick.
    /// - Write values sent from outside the robot (mission goals, teleop
    ///   intent) when they change.
    /// - Read graph outputs by key (visualization, tests).
    pub fn bus(&self) -> &PortBus {
        &self.bus
    }

    /// The inputs declared as sent from outside the robot (goals, teleop
    /// intent), in declaration order. A host can check each one has something
    /// feeding it.
    pub fn outside_inputs(&self) -> &[ChannelKey] {
        &self.outside_inputs
    }

    /// Executes one tick: stamps the bus clock, then runs every rate-due
    /// node in topological order.
    ///
    /// `dt` is the elapsed wall time since the last call (used by
    /// [`RateTimer`]). `now` is the host's monotonic clock, stamped onto the bus
    /// tick-time so producers and `read_fresh` consumers always see the same
    /// clock. `tf` is the transform provider nodes query for the tick.
    pub fn tick(&self, now: MonotonicTime, dt: f64, tf: &dyn TfProvider) {
        self.bus.set_tick_time(now.0);

        // Span only — no event emitted inside. At a default 200 Hz host
        // tick rate, an `info!`/`debug!` per tick would flood the terminal.
        // The span attaches context to any event the nodes themselves emit
        // (errors, warnings) so they're attributable to a tick. `trace`
        // level for the per-node span keeps default `helios=debug` clean;
        // raise to `helios=trace` to see each node firing.
        let _tick_span = debug_span!("pipeline.tick", t = now.0, dt).entered();

        for level in &self.levels {
            for scheduled in level {
                if scheduled.rate_timer.should_fire_and_advance(dt) {
                    let _node_span = trace_span!(
                        "node.execute",
                        name = scheduled.node.name(),
                        id = scheduled.node_id
                    )
                    .entered();
                    scheduled.node.execute(
                        &self.bus,
                        tf,
                        TickContext::new(now, dt, scheduled.node_id, &scheduled.observations),
                    );
                }
            }
        }
    }

    /// Iterates over every output channel produced by the graph, paired with
    /// the name of the node that produces it.
    ///
    /// Order follows the topological build order (level by level, then node
    /// order within each level), so producers always appear before the nodes
    /// that consume their output. Only declared outputs are listed — host-
    /// supplied external channels and channels with no producing node do not
    /// appear here. A node that declares multiple outputs yields one entry per
    /// output, all sharing the same node name.
    ///
    /// Two consumers today: `helios_test` walks these pairs to build the
    /// assertion-target paths (`agent.<agent>.<node>.<channel>`) that tests
    /// reference, and host viz walks them to discover which plural channels
    /// (`Path`, `MapData`) a stack actually declares before reading each by key.
    /// This is metadata about the graph's wiring — to read live values off the
    /// bus, use [`AutonomyPipeline::bus`].
    pub fn channels(&self) -> impl Iterator<Item = (&str, &ChannelKey)> + '_ {
        self.levels.iter().flat_map(|level| {
            level.iter().flat_map(|scheduled| {
                let name = scheduled.node.name();
                scheduled
                    .node
                    .port_descriptor()
                    .outputs()
                    .iter()
                    .map(move |key| (name, key))
            })
        })
    }

    /// The tf-edge output channels this graph declares — every producer's
    /// dual-published transform edge, in topological (producer-first) order.
    ///
    /// This is the drain list a host feeds a `TfService`: it is *derived* from
    /// the nodes' own declared outputs via [`is_tf_edge`], not authored
    /// separately, so coverage is structural — an edge a node emits is an edge
    /// the service drains, with no second list to drift from the graph. A stack
    /// with no edge producer yields an empty list.
    pub fn tf_edge_channels(&self) -> Vec<ChannelKey> {
        self.channels()
            .filter(|(_, key)| is_tf_edge(key))
            .map(|(_, key)| key.clone())
            .collect()
    }

    /// Reads the current ego state, if any node has written one this run.
    ///
    /// Returns `None` during cold-start (before the estimator has produced
    /// its first state) and when no estimator node is present in the graph.
    pub fn read_state(&self) -> Option<Arc<Stamped<FrameAwareState>>> {
        self.bus.read(estimate().into())
    }

    /// Reads the pipeline's actuator terminal — the per-actuator command the
    /// allocator produces, and the host relay consumes.
    ///
    /// Morphology-neutral: [`ActuatorCommand`] is the one universal type every
    /// host applies,
    /// whatever the vehicle. Returns `None` during cold-start (before the
    /// allocator's first output) and when no allocator node is in the graph.
    pub fn read_actuators(&self) -> Option<Arc<Stamped<ActuatorCommand>>> {
        self.bus.read(control::actuators().into())
    }
}

/// A node as the running pipeline holds it: the node plus the state the
/// pipeline keeps for it.
///
/// A field belongs here only when the pipeline gives it to every node, such
/// as rate gating or an observation buffer. State specific to one kind of
/// node stays inside that node.
pub(super) struct ScheduledNode {
    /// Assigned at build in level-major order; stamps the node's bus writes.
    pub(super) node_id: NodeId,
    pub(super) node: Box<dyn PipelineNode>,
    /// Decides each tick whether the node is due. Fires every tick when the
    /// node declares no rate.
    pub(super) rate_timer: RateTimer,
    /// What the node emitted for watchers since the last drain. Nothing in
    /// the pipeline reads it.
    pub(super) observations: NodeObservations,
}

impl ScheduledNode {
    /// Wraps `node` with a timer at its declared rate and an empty buffer
    /// under its name, holding every leaf in its
    /// [`observable_catalog`], none of them watched.
    pub(super) fn new(node_id: NodeId, node: Box<dyn PipelineNode>) -> Self {
        let rate_timer = RateTimer::new(node.port_descriptor().rate());
        let observations = NodeObservations::new(
            node.name(),
            observable_catalog(node.as_ref())
                .into_iter()
                .map(|o| o.leaf_name().clone()),
        );

        Self {
            node_id,
            node,
            rate_timer,
            observations,
        }
    }
}

/// Everything `node` can emit for watchers: the leaves it declares on its
/// descriptor, in declaration order, then the leaves the pipeline adds to every
/// node ([`TICK_DURATION_LEAF`]).
///
/// The pipeline adds a leaf only when it can measure that value the same way
/// for every node, from outside the node. Anything about a node's insides is
/// the node's own declaration. Every reader of a node's catalog (its buffer,
/// the build checks, the startup log) goes through here, so none of them
/// needs to know which leaves the pipeline adds.
pub(crate) fn observable_catalog(node: &dyn PipelineNode) -> Vec<Observable> {
    let mut observables = node.port_descriptor().observables().to_vec();
    observables.push(Observable::new(TICK_DURATION_LEAF, Determinism::WallClock));
    observables
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::{MockNodePortDescriptor, PortDescriptor};

    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";

    /// A node that only declares; it never runs in these tests.
    struct Declaring {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for Declaring {
        fn name(&self) -> &str {
            "declaring"
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    fn declaring(leaves: &[&str]) -> Box<dyn PipelineNode> {
        let descriptor = leaves
            .iter()
            .fold(MockNodePortDescriptor::new(), |builder, leaf| {
                builder.observable(*leaf, Determinism::Reproducible)
            })
            .build();
        Box::new(Declaring { descriptor })
    }

    #[test]
    fn catalog_lists_declared_leaves_then_tick_duration() {
        let node = declaring(&[NIS, DROPPED]);

        assert_eq!(
            observable_catalog(node.as_ref()),
            vec![
                Observable::new(NIS, Determinism::Reproducible),
                Observable::new(DROPPED, Determinism::Reproducible),
                Observable::new(TICK_DURATION_LEAF, Determinism::WallClock),
            ]
        );
    }

    #[test]
    fn catalog_of_a_node_declaring_nothing_is_tick_duration() {
        let node = declaring(&[]);

        assert_eq!(
            observable_catalog(node.as_ref()),
            vec![Observable::new(TICK_DURATION_LEAF, Determinism::WallClock)]
        );
    }

    #[test]
    fn scheduled_node_buffer_holds_the_whole_catalog() {
        let mut scheduled = ScheduledNode::new(0, declaring(&[NIS]));

        assert!(scheduled.observations.set_watched(NIS, true));
        assert!(scheduled.observations.set_watched(TICK_DURATION_LEAF, true));
        assert!(!scheduled.observations.set_watched(DROPPED, true));
    }
}
