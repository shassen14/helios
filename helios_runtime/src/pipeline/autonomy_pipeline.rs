use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

use helios_core::{
    frames::FrameAwareState,
    prelude::{ControlOutput, PlannerGoal},
};

use crate::{
    pipeline::{node::HOST_PRODUCER_ID, rate_gate::RateTimer},
    port::{ChannelKey, PortBus},
    prelude::{
        AgentRuntime, Health, NodeId, PipelineBuildError, PipelineNode, Stamped, TickContext,
    },
};

/// Canonical [`ChannelKey`] instance name for the long-lived mission goal slot.
///
/// `PlannerGoal @ "mission"` is a **state** channel (last-known-good, not
/// cleared each tick). Any code writing to it — `inject_mission_goal`, a
/// Zenoh bridge, the mission layer — must use this constant rather than a
/// duplicated literal so the contract stays in one place.
pub const MISSION_GOAL_INSTANCE: &str = "mission";

/// Constructs a [`AutonomyPipeline`] from a set of [`PipelineNode`]s and the
/// channels supplied from outside the graph.
///
/// Build sequence:
/// 1. Register every node with [`add_node`](Self::add_node).
/// 2. Declare channels written by sensor tick systems via
///    [`with_sensor_signals`](Self::with_sensor_signals). These slots are
///    cleared each tick by [`PortBus::clear_signals`].
/// 3. Declare channels written by external systems (mission layer, Zenoh,
///    operator UI) via [`with_host_states`](Self::with_host_states). These
///    persist across ticks.
/// 4. Call [`build`](Self::build) — returns either a fully validated
///    pipeline or every detected error.
///
/// Both `with_sensor_signals` and `with_host_states` seed the topological
/// sort so a node consuming an externally-provided channel doesn't fail
/// with [`PipelineBuildError::UnsatisfiedInput`]. The split between them
/// only affects `PortBus`'s signal-clear list: sensor signals get cleared
/// each tick; host states do not.
pub struct PipelineBuilder {
    nodes: Vec<Box<dyn PipelineNode>>,
    sensor_signal_keys: Vec<ChannelKey>,
    host_state_keys: Vec<ChannelKey>,
}

impl Default for PipelineBuilder {
    fn default() -> Self {
        PipelineBuilder::new()
    }
}

impl PipelineBuilder {
    /// Creates an empty builder. Add nodes and declare external channels
    /// before calling [`build`](Self::build).
    pub fn new() -> Self {
        PipelineBuilder {
            nodes: vec![],
            sensor_signal_keys: vec![],
            host_state_keys: vec![],
        }
    }

    /// Registers one node. Order does not matter — the topological sort
    /// places each node in the correct level based on its declared inputs
    /// and outputs.
    pub fn add_node(mut self, node: Box<dyn PipelineNode>) -> Self {
        self.nodes.push(node);
        self
    }

    /// Declares channels written by sensor tick systems each tick.
    ///
    /// These keys are passed to [`PortBus::clear_signals`] so the slot is
    /// wiped at the start of every tick — sensor batches must not carry
    /// across ticks.
    pub fn with_sensor_signals(mut self, keys: Vec<ChannelKey>) -> Self {
        self.sensor_signal_keys = keys;
        self
    }

    /// Declares persistent channels written by external systems
    /// (mission layer, Zenoh bridge, operator UI).
    ///
    /// Unlike sensor signals, these slots are **not** cleared each tick —
    /// the host writes once when the value changes, consumers read the
    /// last-known-good value on every subsequent tick. This is the slot
    /// semantics needed for mission goals, mode flags, and parameter
    /// overrides.
    pub fn with_host_states(mut self, keys: Vec<ChannelKey>) -> Self {
        self.host_state_keys = keys;
        self
    }

    /// Validates the registered nodes and builds a [`AutonomyPipeline`].
    ///
    /// All errors are collected — `build` never short-circuits. The
    /// returned [`Vec`] contains every issue found, in this order:
    /// 1. [`PipelineBuildError::MultipleProducers`] — two nodes declared
    ///    the same output channel.
    /// 2. [`PipelineBuildError::UnsatisfiedInput`] — a required input has
    ///    no producer and is not in the sensor/host declarations.
    /// 3. [`PipelineBuildError::Cycle`] — a remaining sub-graph has every
    ///    input satisfied only by other stranded nodes' outputs.
    ///
    /// On success, [`NodeId`]s are assigned in level-major order starting
    /// at `0`, indexing into the pipeline's rate-timer array.
    pub fn build(self) -> Result<AutonomyPipeline, Vec<PipelineBuildError>> {
        let mut errors: Vec<PipelineBuildError> = vec![];

        // First pass: single-producer check. For each output channel of
        // each node, record the first node that claims it; any later
        // claimant is a conflict.
        let mut producer_of: HashMap<ChannelKey, String> = HashMap::new();
        for node in &self.nodes {
            let node_name = node.name();
            for output in &node.port_descriptor().outputs {
                if let Some(prev_name) = producer_of.insert(output.clone(), node_name.to_string()) {
                    let error = PipelineBuildError::MultipleProducers {
                        channel: output.clone(),
                        first_node: prev_name,
                        second_node: node_name.to_string(),
                    };
                    errors.push(error);
                }
            }
        }

        // Seed `produced` with channels supplied from outside the graph.
        // The topological sort treats them as already-satisfied so any
        // consumer of these channels doesn't trip UnsatisfiedInput.
        let mut produced: HashSet<ChannelKey> = HashSet::new();
        produced.extend(self.sensor_signal_keys.iter().cloned());
        produced.extend(self.host_state_keys.iter().cloned());

        // Kahn's algorithm (level-by-level form). Each iteration pulls out
        // every node whose required inputs are already produced, assigns
        // it a NodeId, and pushes it into the current level. The level's
        // outputs then enter `produced` so the next iteration can advance.
        let mut remaining: Vec<Box<dyn PipelineNode>> = self.nodes;
        let mut levels: Vec<Vec<(NodeId, Box<dyn PipelineNode>)>> = Vec::new();
        let mut next_id: NodeId = 0;

        while !remaining.is_empty() {
            let (ready, still_waiting): (Vec<_>, Vec<_>) =
                remaining.into_iter().partition(|node| {
                    node.port_descriptor()
                        .required_inputs
                        .iter()
                        .all(|channel| produced.contains(channel))
                });

            remaining = still_waiting;

            // If no node is ready but `remaining` is non-empty, the sort
            // is stuck. Either an input has no producer anywhere, or two
            // or more nodes are mutually waiting on each other (cycle).
            if ready.is_empty() {
                // Channels that *would* exist if the sort could continue.
                let mut pending_outputs: HashSet<ChannelKey> = HashSet::new();
                for node in &remaining {
                    for channel in &node.port_descriptor().outputs {
                        pending_outputs.insert(channel.clone());
                    }
                }

                // Cycle pass: a remaining node whose required inputs are
                // entirely covered by `produced ∪ pending_outputs` is
                // blocked purely by other stranded nodes — that is a
                // cycle. One Cycle error is emitted regardless of how
                // many nodes participate.
                let is_cycle_detected = remaining.iter().any(|node| {
                    node.port_descriptor()
                        .required_inputs
                        .iter()
                        .all(|channel| {
                            produced.contains(channel) || pending_outputs.contains(channel)
                        })
                });

                if is_cycle_detected {
                    errors.push(PipelineBuildError::Cycle);
                }

                // Unsatisfied-input pass: any required input not covered
                // by `produced` *or* `pending_outputs` is genuinely
                // missing — no node anywhere will ever produce it.
                for node in &remaining {
                    for channel in &node.port_descriptor().required_inputs {
                        if !produced.contains(channel) && !pending_outputs.contains(channel) {
                            errors.push(PipelineBuildError::UnsatisfiedInput {
                                node_name: node.name().to_string(),
                                channel: channel.clone(),
                            });
                        }
                    }
                }

                break;
            }

            // Promote this level's outputs into `produced` so the next
            // iteration can advance.
            produced.extend(
                ready
                    .iter()
                    .flat_map(|node| node.port_descriptor().outputs.iter())
                    .cloned(),
            );

            // Assign NodeIds in level-major order as we go — this is the
            // same order the rate-timer array will be indexed by at tick
            // time, so the two stay in lockstep without a second pass.
            let mut level: Vec<(NodeId, Box<dyn PipelineNode>)> = Vec::with_capacity(ready.len());
            for node in ready {
                level.push((next_id, node));
                next_id += 1;
            }

            levels.push(level);
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        // Bus slots are allocated from the union of all node descriptors.
        // A host-state channel gets its slot via the consuming node's
        // `required_inputs` (or `optional_inputs`) — no need to seed slots
        // from `host_state_keys` separately. If no node consumes a host
        // channel, the host write returns `UnknownChannel`, which is the
        // correct outcome.
        let descriptor_iter = levels
            .iter()
            .flat_map(|level| level.iter().map(|(_, node)| node.port_descriptor()));

        // Only sensor signals go into the bus's signal-clear list. Host
        // states must persist across ticks.
        let bus = PortBus::new(descriptor_iter, self.sensor_signal_keys);

        // One timer per node, indexed by NodeId (which matches level-major
        // iteration order below).
        let mut rate_timers: Vec<RateTimer> = Vec::with_capacity(next_id as usize);
        for level in &levels {
            for (_, node) in level {
                rate_timers.push(RateTimer::new(node.port_descriptor().rate));
            }
        }

        Ok(AutonomyPipeline {
            levels,
            bus,
            rate_timers,
        })
    }
}

/// A built, validated autonomy pipeline.
///
/// Constructed only via [`PipelineBuilder::build`]. After construction
/// the topology is fixed; the only mutable state is the bus contents and
/// per-node timer counters, both of which use interior mutability so
/// [`tick`](Self::tick) can take `&self`.
pub struct AutonomyPipeline {
    /// Nodes grouped by topological level, in execution order. Each entry
    /// pairs a node with its build-time-assigned [`NodeId`].
    levels: Vec<Vec<(NodeId, Box<dyn PipelineNode>)>>,
    /// Typed blackboard used for all intra-pipeline data exchange. Also
    /// the only way external systems (host, mission layer) write values
    /// into the graph.
    bus: PortBus,
    /// Per-node rate gating, indexed by [`NodeId`]. A node with
    /// `rate: None` fires every tick.
    rate_timers: Vec<RateTimer>,
}

impl AutonomyPipeline {
    /// Returns a reference to the [`PortBus`] for direct read/write access.
    ///
    /// External callers use this to:
    /// - Clear and re-write sensor signals at the start of each tick.
    /// - Inject host-state values (mission goals, mode flags, etc.) when
    ///   they change, via [`PortBus::write`].
    pub fn bus(&self) -> &PortBus {
        &self.bus
    }

    /// Executes one tick: stamps the bus clock, then runs every rate-due
    /// node in topological order.
    ///
    /// `tick` deliberately does **not** call [`PortBus::clear_signals`].
    /// The host owns the per-tick sequence so it can also be driven from
    /// non-Bevy contexts (hardware loop, replay):
    ///
    /// ```text
    /// pipeline.bus().clear_signals();
    /// // host writes fresh sensor data into the bus
    /// pipeline.tick(runtime, dt);
    /// ```
    ///
    /// `dt` is the elapsed wall time since the last call (used by
    /// [`RateTimer`]). The bus tick-time is sourced from
    /// [`AgentRuntime::now`] so producers and `read_fresh` consumers
    /// always see the same clock.
    pub fn tick(&self, runtime: &dyn AgentRuntime, dt: f64) {
        let now = runtime.now();
        self.bus.set_tick_time(now.0);

        for level in &self.levels {
            for (node_id, node) in level {
                if self.rate_timers[*node_id as usize].should_fire_and_advance(dt) {
                    node.execute(
                        &self.bus,
                        runtime,
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

    /// Writes a mission goal to the canonical `PlannerGoal @ "mission"`
    /// channel.
    ///
    /// This is a thin wrapper over [`PortBus::write`] for the one channel
    /// that has a named in-process helper because of how often it's used.
    /// Any other host-state channel uses `pipeline.bus().write(...)`
    /// directly.
    ///
    /// The write is silently dropped if no node in the graph consumes the
    /// mission channel (the bus has no slot for it). This is the documented
    /// behavior for unused channels — see `dag_engine_plan.md` Decision 1.
    pub fn inject_mission_goal(&self, goal: PlannerGoal, runtime: &dyn AgentRuntime) {
        let goal_stamped = Stamped {
            value: goal,
            timestamp: runtime.now(),
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        let _ = self.bus.write(
            ChannelKey::named::<PlannerGoal>(MISSION_GOAL_INSTANCE),
            goal_stamped,
        );
    }

    /// Reads the current ego state, if any node has written one this run.
    ///
    /// Returns `None` during cold-start (before the estimator has produced
    /// its first state) and when no estimator node is present in the graph.
    pub fn read_state(&self) -> Option<Arc<Stamped<FrameAwareState>>> {
        self.bus.read(ChannelKey::of::<FrameAwareState>())
    }

    /// Reads the current control output, if any controller node has
    /// written one this run.
    pub fn read_control(&self) -> Option<Arc<Stamped<ControlOutput>>> {
        self.bus.read(ChannelKey::of::<ControlOutput>())
    }
}
