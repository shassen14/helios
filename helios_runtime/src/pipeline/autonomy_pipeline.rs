use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

use tracing::{debug_span, info, trace_span};

use helios_core::{
    frames::FrameAwareState,
    mapping::MapData,
    planning::types::Path,
    prelude::{ControlOutput, PlannerGoal},
};

use crate::{
    pipeline::{key_format::format_key_short, node::HOST_PRODUCER_ID, rate_gate::RateTimer},
    port::{ChannelKey, InternalChannel, PortBus},
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
/// 2. Declare channels written from outside the graph — sensor tick systems,
///    mission layer, Zenoh bridge, operator UI — via
///    [`with_external_channels`](Self::with_external_channels). These seed
///    the topological sort so consumers don't trip
///    [`PipelineBuildError::UnsatisfiedInput`].
/// 3. Call [`build`](Self::build) — returns either a fully validated
///    pipeline or every detected error.
///
/// All bus slots use last-known-good semantics; nothing is cleared per tick.
/// Consumers that must process each reading at most once track their own
/// last-seen [`Stamped::timestamp`] internally.
pub struct PipelineBuilder {
    nodes: Vec<Box<dyn PipelineNode>>,
    external_channel_keys: Vec<ChannelKey>,
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
            external_channel_keys: vec![],
        }
    }

    /// Registers one node. Order does not matter — the topological sort
    /// places each node in the correct level based on its declared inputs
    /// and outputs.
    pub fn add_node(mut self, node: Box<dyn PipelineNode>) -> Self {
        self.nodes.push(node);
        self
    }

    /// Declares channels written from outside the graph.
    ///
    /// These keys seed the topological sort as already-satisfied so a
    /// consumer of an externally-supplied channel doesn't fail with
    /// [`PipelineBuildError::UnsatisfiedInput`]. Slot allocation in
    /// [`PortBus`] still comes from node descriptors — if no node consumes
    /// a channel, the host write returns [`crate::port::ChannelError::UnknownChannel`].
    pub fn with_external_channels(mut self, keys: Vec<ChannelKey>) -> Self {
        self.external_channel_keys = keys;
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
        produced.extend(self.external_channel_keys.iter().cloned());

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
                    let participants = remaining
                        .iter()
                        .filter(|node| {
                            node.port_descriptor()
                                .required_inputs
                                .iter()
                                .all(|channel| {
                                    produced.contains(channel) || pending_outputs.contains(channel)
                                })
                        })
                        .map(|node| node.name().to_string())
                        .collect();
                    errors.push(PipelineBuildError::Cycle { participants });
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
        // An external channel gets its slot via the consuming node's
        // `required_inputs` (or `optional_inputs`) — no need to seed slots
        // from `external_channel_keys` separately. If no node consumes an
        // external channel, the host write returns `UnknownChannel`, which
        // is the correct outcome.
        let descriptor_iter = levels
            .iter()
            .flat_map(|level| level.iter().map(|(_, node)| node.port_descriptor()));

        let bus = PortBus::new(descriptor_iter);

        // One timer per node, indexed by NodeId (which matches level-major
        // iteration order below).
        let mut rate_timers: Vec<RateTimer> = Vec::with_capacity(next_id as usize);
        for level in &levels {
            for (_, node) in level {
                rate_timers.push(RateTimer::new(node.port_descriptor().rate));
            }
        }

        log_resolved_dag(&levels);

        Ok(AutonomyPipeline {
            levels,
            bus,
            rate_timers,
        })
    }
}

/// One-shot startup dump of the resolved DAG. Emits at `info` so it's
/// visible with the default `helios=info` filter; nothing else in the
/// per-tick path emits at that level, so this stays a single block.
fn log_resolved_dag(levels: &[Vec<(NodeId, Box<dyn PipelineNode>)>]) {
    info!(
        target: "helios_runtime::pipeline",
        levels = levels.len(),
        nodes = levels.iter().map(|level| level.len()).sum::<usize>(),
        "resolved autonomy pipeline",
    );

    for (level_idx, level) in levels.iter().enumerate() {
        for (node_id, node) in level {
            let descriptor = node.port_descriptor();
            let rate = match descriptor.rate {
                Some(hz) => format!("{hz} Hz"),
                None => "every tick".to_string(),
            };
            let inputs = format_keys(&descriptor.required_inputs);
            let optional = format_keys(&descriptor.optional_inputs);
            let outputs = format_keys(&descriptor.outputs);
            info!(
                target: "helios_runtime::pipeline",
                level = level_idx,
                id = *node_id,
                name = node.name(),
                rate = %rate,
                inputs = %inputs,
                optional_inputs = %optional,
                outputs = %outputs,
                "  node",
            );
        }
    }
}

fn format_keys(keys: &[ChannelKey]) -> String {
    if keys.is_empty() {
        return "[]".to_string();
    }
    let joined = keys
        .iter()
        .map(format_key_short)
        .collect::<Vec<_>>()
        .join(", ");
    format!("[{joined}]")
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
    /// `dt` is the elapsed wall time since the last call (used by
    /// [`RateTimer`]). The bus tick-time is sourced from
    /// [`AgentRuntime::now`] so producers and `read_fresh` consumers
    /// always see the same clock.
    pub fn tick(&self, runtime: &dyn AgentRuntime, dt: f64) {
        let now = runtime.now();
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
            InternalChannel::named::<PlannerGoal>(MISSION_GOAL_INSTANCE).into(),
            goal_stamped,
        );
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
    pub fn read_control(&self) -> Option<Arc<Stamped<ControlOutput>>> {
        self.bus.read(InternalChannel::of::<ControlOutput>().into())
    }

    /// Reads any [`Path`] currently on the bus, ignoring the planner instance
    /// name. Convenience for single-planner stacks and debug visualization.
    /// For multi-planner stacks, read the specific planner's channel via
    /// `bus().read::<Path>(ChannelKey::named::<Path>(planner_name))`.
    pub fn read_any_path(&self) -> Option<Arc<Stamped<Path>>> {
        self.bus.read_any::<Path>()
    }

    /// Reads any [`MapData`] currently on the bus, ignoring the layer
    /// instance name. Convenience for debug visualization.
    pub fn read_any_map(&self) -> Option<Arc<Stamped<MapData>>> {
        self.bus.read_any::<MapData>()
    }

    /// Reads the currently-injected mission goal, if any.
    pub fn read_mission_goal(&self) -> Option<Arc<Stamped<PlannerGoal>>> {
        self.bus
            .read(InternalChannel::named::<PlannerGoal>(MISSION_GOAL_INSTANCE).into())
    }
}
