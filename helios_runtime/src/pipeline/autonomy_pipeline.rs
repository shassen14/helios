//! [`AutonomyPipeline`]: a built graph of nodes over a bus, run one tick at a
//! time. Only the builder in [`build`](super::build) constructs one.

use crate::{
    channels::{control, tf::is_tf_edge},
    pipeline::rate_gate::RateTimer,
    port::{ChannelKey, InternalChannel, PortBus},
    prelude::{PipelineNode, Stamped, TickContext},
    NodeId,
};

use helios_core::{
    control::{actuators::ActuatorCommand, commands::BodyTwist},
    prelude::{MonotonicTime, TfProvider},
    spatial::FrameAwareState,
};

use std::sync::Arc;
use tracing::{debug_span, trace_span};

/// A built, validated autonomy pipeline.
///
/// Constructed only via [`PipelineBuilder::build`](crate::PipelineBuilder::build). After construction
/// the topology is fixed; the only mutable state is the bus contents and
/// per-node timer counters, both of which use interior mutability so
/// [`tick`](Self::tick) can take `&self`.
pub struct AutonomyPipeline {
    /// Nodes grouped by topological level, in execution order. Each entry
    /// pairs a node with its build-time-assigned [`NodeId`].
    pub(super) levels: Vec<Vec<(NodeId, Box<dyn PipelineNode>)>>,
    /// Typed blackboard used for all intra-pipeline data exchange. Also
    /// the only way values from outside the graph enter it: the host writes
    /// body measurements and operator or mission inputs here.
    pub(super) bus: PortBus,
    /// Per-node rate gating, indexed by [`NodeId`]. A node with
    /// `rate: None` fires every tick.
    pub(super) rate_timers: Vec<RateTimer>,
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
            for (node_id, node) in level {
                if self.rate_timers[*node_id as usize].should_fire_and_advance(dt) {
                    let _node_span =
                        trace_span!("node.execute", name = node.name(), id = *node_id).entered();
                    node.execute(
                        &self.bus,
                        tf,
                        TickContext {
                            now,
                            dt,
                            node_id: *node_id,
                        },
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
            level.iter().flat_map(|(_, node)| {
                let name = node.name();
                node.port_descriptor()
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
        self.bus
            .read(InternalChannel::of::<FrameAwareState>().into())
    }

    /// Reads the current control output, if any controller node has
    /// written one this run.
    ///
    /// This canonical accessor names one concrete command type. Today that is
    /// [`BodyTwist`] — the command the current controller family emits — so it is
    /// morphology-specific: a controller whose `Out` is not `BodyTwist` publishes
    /// fine on the bus (the node is generic over `C::Out`) but is not visible
    /// through this accessor. The actuator terminal makes the canonical control
    /// output a single universal command type, at which point this accessor stops
    /// being morphology-specific. Read other command channels by name via
    /// [`bus`](Self::bus)`().read::<T>(key)` in the meantime.
    pub fn read_control(&self) -> Option<Arc<Stamped<BodyTwist>>> {
        self.bus.read(control::command::<BodyTwist>().into())
    }

    /// Reads the pipeline's actuator terminal — the per-actuator command the
    /// allocator produces, and the host relay consumes.
    ///
    /// Unlike [`read_control`](Self::read_control), this is morphology-neutral:
    /// [`ActuatorCommand`] is the one universal type every host applies,
    /// whatever the vehicle. Returns `None` during cold-start (before the
    /// allocator's first output) and when no allocator node is in the graph.
    pub fn read_actuators(&self) -> Option<Arc<Stamped<ActuatorCommand>>> {
        self.bus.read(control::actuators().into())
    }
}
