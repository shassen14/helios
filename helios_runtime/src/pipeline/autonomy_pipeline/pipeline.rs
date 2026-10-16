//! [`AutonomyPipeline`]: a built graph of nodes over a bus, run one tick at a
//! time. Only the builder in [`build`](crate::pipeline::build) constructs one.

use super::scheduled::{ScheduledNode, TICK_DURATION_LEAF};

use crate::{
    channels::{control, estimate::estimate},
    port::{ChannelDescription, ChannelKey, InputPort, MountReading, PortBus, Writer},
    prelude::{Stamped, TickContext},
};

use helios_core::{
    control::actuators::ActuatorCommand,
    prelude::{AgentId, MonotonicTime, TfProvider},
    spatial::{transforms::Convention, FrameAwareState, FrameId},
};

use std::{sync::Arc, time::Instant};
use tracing::{debug_span, trace_span};

/// A built, validated autonomy pipeline.
///
/// Constructed only via [`PipelineBuilder::build`](crate::PipelineBuilder::build). After construction
/// the topology is fixed; the only mutable state is the bus contents, the
/// per-node timer counters, and the per-node observation buffers, all of
/// which use interior mutability so [`tick`](Self::tick) can take `&self`.
pub struct AutonomyPipeline {
    /// The agent whose brain this is, by its config name. Every frame the
    /// pipeline reads is scoped by it, and every recorded key names it. The
    /// agent owns the pipeline; this is only its name.
    pub(in crate::pipeline) agent: AgentId,
    /// Nodes grouped by topological level, in execution order, each with
    /// the state the pipeline keeps for it. Walking levels in order visits
    /// nodes in [`NodeId`] order.
    pub(in crate::pipeline) levels: Vec<Vec<ScheduledNode>>,
    /// Typed blackboard used for all intra-pipeline data exchange. Also
    /// the only way values from outside the graph enter it: the host writes
    /// body measurements and operator or mission inputs here.
    pub(in crate::pipeline) bus: PortBus,
    /// Every channel some writer declared, kept from the build: the body's,
    /// then the outside inputs, then the nodes' outputs.
    pub(in crate::pipeline) descriptions: Vec<ChannelDescription>,
    /// The frame the estimate holds the body's pose in, when the stack names
    /// an estimate.
    pub(in crate::pipeline) estimate_root: Option<FrameId>,
}

impl AutonomyPipeline {
    /// The agent whose brain this is.
    pub fn agent(&self) -> &AgentId {
        &self.agent
    }

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

    /// Every channel some writer declared: the body's, then the outside
    /// inputs, then the nodes' outputs. Channels only read are not listed;
    /// the build refuses a read without a writer.
    pub fn descriptions(&self) -> &[ChannelDescription] {
        &self.descriptions
    }

    /// The inputs declared as sent from outside the robot (goals, teleop
    /// intent), in declaration order. A host can check each one has something
    /// feeding it.
    pub fn outside_inputs(&self) -> impl Iterator<Item = &ChannelKey> {
        self.descriptions
            .iter()
            .filter(|description| *description.writer() == Writer::Outside)
            .map(ChannelDescription::key)
    }

    /// The frame the estimate holds the body's pose in: the root of the
    /// estimator the stack names as its estimate, the parent of the edge its
    /// relay publishes to `base_link`. `None` when the stack names no
    /// estimate. A host tracks and draws the estimate under this frame.
    pub fn estimate_root(&self) -> Option<&FrameId> {
        self.estimate_root.as_ref()
    }

    /// The mounts of the channels some node reads, each frame once, in
    /// execution order, with every axis convention a node reads it in through
    /// the tf tree. A host checks each sits below `base_link` in the tree it
    /// serves the pipeline, held there in those axes, once its static feed is
    /// complete. A channel no node reads is left out: an unused sensor's frame
    /// is not the pipeline's concern.
    pub fn read_mounts(&self) -> Vec<ReadMount> {
        let mut mounts: Vec<ReadMount> = Vec::new();
        for scheduled in self.levels.iter().flatten() {
            let descriptor = scheduled.node.port_descriptor();
            for channel in descriptor.inputs().map(InputPort::channel) {
                let mount = self
                    .descriptions
                    .iter()
                    .find(|description| description.key() == channel)
                    .and_then(ChannelDescription::mount);
                let Some(mount) = mount else {
                    continue;
                };
                let index = match mounts.iter().position(|read| &read.frame == mount) {
                    Some(index) => index,
                    None => {
                        mounts.push(ReadMount {
                            frame: mount.clone(),
                            read_in: Vec::new(),
                        });
                        mounts.len() - 1
                    }
                };
                let read_in = descriptor
                    .mount_readings()
                    .iter()
                    .filter(|reading| reading.channel() == channel)
                    .map(MountReading::convention);
                for convention in read_in {
                    if !mounts[index].read_in.contains(&convention) {
                        mounts[index].read_in.push(convention);
                    }
                }
            }
        }
        mounts
    }

    /// Executes one tick: stamps the bus clock, then runs every rate-due
    /// node in topological order.
    ///
    /// `dt` is the elapsed wall time since the last call (used by
    /// [`RateTimer`](crate::pipeline::rate_gate::RateTimer)). `now` is the
    /// host's monotonic clock, stamped onto the bus tick-time so producers and
    /// `read_fresh` consumers always see the same clock. `tf` is the transform
    /// provider nodes query for the tick.
    ///
    /// When a node's [`TICK_DURATION_LEAF`] is watched, its run is timed on a
    /// wall clock and recorded in seconds, stamped with `now`. When a channel
    /// instance it writes is watched, the value the node wrote there is
    /// recorded under the channel's key once per run that wrote it, stamped
    /// with the value's own time. A node's run records its channels before
    /// its duration, and both after what it emitted itself.
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

                    // Read the watched taps' versions before the clock starts,
                    // so the duration measures the node alone.
                    let taps = scheduled.watched_tap_versions(&self.bus);

                    // Time the run only when a watcher asked for it, so an
                    // unwatched node pays one check and never reads the clock.
                    let timing = scheduled
                        .observations
                        .watched_leaf(TICK_DURATION_LEAF)
                        .map(|leaf| (leaf, Instant::now()));

                    scheduled.node.execute(
                        &self.bus,
                        tf,
                        TickContext::new(now, dt, scheduled.node_id, &scheduled.observations),
                    );

                    let elapsed = timing.map(|(leaf, start)| (leaf, start.elapsed()));

                    scheduled.record_tap_writes(&self.bus, taps);
                    if let Some((leaf, elapsed)) = elapsed {
                        scheduled
                            .observations
                            .record(leaf, now, elapsed.as_secs_f64().into());
                    }
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
    /// Host viz walks them to discover which plural channels
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
                    .map(move |key| (name, key))
            })
        })
    }

    /// Reads the current ego state, if any node has written one this run.
    ///
    /// Returns `None` during cold-start (before the estimator has produced
    /// its first state) and when no estimator node is present in the graph.
    pub fn read_state(&self) -> Option<Arc<Stamped<FrameAwareState>>> {
        self.bus.read(estimate())
    }

    /// Reads the pipeline's actuator terminal — the per-actuator command the
    /// allocator produces, and the host relay consumes.
    ///
    /// Morphology-neutral: [`ActuatorCommand`] is the one universal type every
    /// host applies,
    /// whatever the vehicle. Returns `None` during cold-start (before the
    /// allocator's first output) and when no allocator node is in the graph.
    pub fn read_actuators(&self) -> Option<Arc<Stamped<ActuatorCommand>>> {
        self.bus.read(control::actuators())
    }
}

/// A frame some node reads a channel at, from
/// [`AutonomyPipeline::read_mounts`]. A plain report with no invariant to
/// guard, so its fields are open, unlike the [`MountReading`] it is gathered
/// from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadMount {
    /// The mount, a frame some read channel names as its writer's.
    pub frame: FrameId,
    /// Every axis convention a node reads the frame in through the tf tree,
    /// each once; empty when no node resolves it.
    pub read_in: Vec<Convention>,
}
