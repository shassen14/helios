use crate::{
    channels::{control, tf::is_tf_edge},
    pipeline::{key_format::format_key_short, rate_gate::RateTimer},
    port::{ChannelKey, ChannelKind, InternalChannel, PortBus},
    prelude::{PipelineNode, Stamped, TickContext},
    BodyCapabilities, NodeId, PipelineBuildError, Supplier,
};

use helios_core::{
    control::{actuators::ActuatorCommand, commands::BodyTwist},
    prelude::{MonotonicTime, TfProvider},
    spatial::FrameAwareState,
};

use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};
use tracing::{debug_span, info, trace_span};

/// Constructs a [`AutonomyPipeline`] from a set of [`PipelineNode`]s and the
/// [`BodyCapabilities`] of the body the graph runs against.
///
/// Build sequence:
/// 1. Register every node with [`add_node`](Self::add_node).
/// 2. Declare the channels the body publishes — sensor channels, `oracle/*`
///    reference channels, `health/*` status — via
///    [`with_body_capabilities`](Self::with_body_capabilities). These count
///    as supplied, so consumers don't trip
///    [`PipelineBuildError::UnsatisfiedInput`].
/// 3. Declare the inputs sent from outside the robot — goals, teleop intent —
///    via [`with_outside_inputs`](Self::with_outside_inputs). These count as
///    supplied the same way.
/// 4. Call [`build`](Self::build) — returns either a fully validated
///    pipeline or every detected error.
///
/// All bus slots use last-known-good semantics; nothing is cleared per tick.
/// Consumers that must process each reading at most once track their own
/// last-seen [`Stamped::timestamp`] internally.
pub struct PipelineBuilder {
    nodes: Vec<Box<dyn PipelineNode>>,
    capabilities: BodyCapabilities,
    outside_inputs: Vec<ChannelKey>,
}

impl Default for PipelineBuilder {
    fn default() -> Self {
        PipelineBuilder::new()
    }
}

impl PipelineBuilder {
    /// Creates an empty builder with a default (passive, empty)
    /// [`BodyCapabilities`]. Add nodes and declare the body's published
    /// channels via [`with_body_capabilities`](Self::with_body_capabilities)
    /// before calling [`build`](Self::build).
    pub fn new() -> Self {
        PipelineBuilder {
            nodes: vec![],
            capabilities: BodyCapabilities::default(),
            outside_inputs: vec![],
        }
    }

    /// Registers one node. Order does not matter — the topological sort
    /// places each node in the correct level based on its declared inputs
    /// and outputs.
    pub fn add_node(mut self, node: Box<dyn PipelineNode>) -> Self {
        self.nodes.push(node);
        self
    }

    /// Declares the body's I/O surface — the channels it publishes
    /// (sensor signals, oracle reference channels, health) and whether it consumes
    /// control. Each published channel counts as supplied in
    /// [`build`](Self::build), so consumers, required or optional, don't trip
    /// [`PipelineBuildError::UnsatisfiedInput`]; an unmet `Oracle`/`Health`
    /// input instead surfaces as
    /// [`PipelineBuildError::UnsatisfiedBodyCapabilities`], naming the body
    /// that failed to advertise it.
    pub fn with_body_capabilities(mut self, capabilities: BodyCapabilities) -> Self {
        self.capabilities = capabilities;
        self
    }

    /// Declares the inputs an operator or mission system sends the graph:
    /// goals, teleop intent. They are not body channels, so they are kept apart
    /// from [`with_body_capabilities`](Self::with_body_capabilities).
    ///
    /// Each key counts as already written in [`build`](Self::build), as a body
    /// channel does. The declaration is taken on trust: nothing here checks
    /// that the host actually feeds these keys. The built pipeline keeps the
    /// list, readable through [`AutonomyPipeline::outside_inputs`].
    pub fn with_outside_inputs(mut self, outside_inputs: Vec<ChannelKey>) -> Self {
        self.outside_inputs = outside_inputs;
        self
    }

    /// Validates the registered nodes and builds a [`AutonomyPipeline`].
    ///
    /// Errors are checked in two stages, and every error within a stage is
    /// collected:
    /// 1. Wiring. Node names are unique, and every input, required or
    ///    optional, has exactly one supplier: a node output, a body channel,
    ///    or a declared outside input.
    ///    - [`PipelineBuildError::DuplicateNodeName`] — two nodes share a
    ///      name.
    ///    - [`PipelineBuildError::MultipleSuppliers`] — two suppliers
    ///      provide the same channel.
    ///    - [`PipelineBuildError::UnsatisfiedInput`] /
    ///      [`PipelineBuildError::UnsatisfiedBodyCapabilities`] — an input
    ///      has no supplier.
    /// 2. Ordering, run only when wiring is clean, so a missing input never
    ///    also shows up as a cycle.
    ///    - [`PipelineBuildError::Cycle`] — a remaining sub-graph has every
    ///      required input satisfied only by other stranded nodes' outputs.
    ///
    /// Within a level, nodes are sorted by name, so levels and [`NodeId`]s
    /// do not depend on the order nodes were added.
    ///
    /// On success, [`NodeId`]s are assigned in level-major order starting
    /// at `0`, indexing into the pipeline's rate-timer array.
    pub fn build(self) -> Result<AutonomyPipeline, Vec<PipelineBuildError>> {
        let mut errors: Vec<PipelineBuildError> = vec![];

        // Name pass: errors, logs and the within-level order all identify a
        // node by name, so a shared name is reported once per name.
        let mut seen_names: HashSet<&str> = HashSet::new();
        let mut reported_names: HashSet<&str> = HashSet::new();
        for node in &self.nodes {
            let name = node.name();
            if !seen_names.insert(name) && reported_names.insert(name) {
                errors.push(PipelineBuildError::DuplicateNodeName {
                    name: name.to_string(),
                });
            }
        }

        // Supplier pass: record who supplies each channel. The body and the
        // outside inputs go in first, so a conflict names the outside-world
        // supplier before the node. Any second supplier of a key is an error.
        let mut supplier_of: HashMap<ChannelKey, Supplier> = HashMap::new();
        let body_supplies = self.capabilities.publishes.iter().map(|published| {
            (
                published.key.clone(),
                Supplier::Body(self.capabilities.name.clone()),
            )
        });
        let outside_supplies = self
            .outside_inputs
            .iter()
            .map(|key| (key.clone(), Supplier::OutsideInput));
        let node_supplies = self.nodes.iter().flat_map(|node| {
            node.port_descriptor()
                .outputs()
                .iter()
                .map(|output| (output.clone(), Supplier::Node(node.name().to_string())))
        });
        for (channel, supplier) in body_supplies.chain(outside_supplies).chain(node_supplies) {
            if let Some(first) = supplier_of.get(&channel) {
                errors.push(PipelineBuildError::MultipleSuppliers {
                    channel,
                    first: first.clone(),
                    second: supplier,
                });
            } else {
                supplier_of.insert(channel, supplier);
            }
        }

        // Existence pass: every input, required or optional, needs a
        // supplier. Optional only means the node runs without a value.
        for node in &self.nodes {
            for input in node.port_descriptor().inputs() {
                let channel = input.channel();
                if supplier_of.contains_key(channel) {
                    continue;
                }
                let error = match channel.kind() {
                    ChannelKind::Sensor | ChannelKind::Internal => {
                        PipelineBuildError::UnsatisfiedInput {
                            node_name: node.name().to_string(),
                            channel: channel.clone(),
                            need: input.need(),
                        }
                    }
                    // Only a body supplies oracle and health channels, so a
                    // missing one is the body's gap and the error names it.
                    ChannelKind::Health | ChannelKind::Oracle => {
                        PipelineBuildError::UnsatisfiedBodyCapabilities {
                            node_name: node.name().to_string(),
                            channel_key: channel.clone(),
                            body: self.capabilities.name.clone(),
                            need: input.need(),
                        }
                    }
                };
                errors.push(error);
            }
        }

        // Stop before ordering: a node downstream of a missing input would
        // otherwise be reported as a cycle member too.
        if !errors.is_empty() {
            return Err(errors);
        }

        // Seed `produced` with channels supplied from outside the graph, so
        // the sort treats them as already written.
        let mut produced: HashSet<ChannelKey> = HashSet::new();
        produced.extend(self.capabilities.publishes.iter().map(|p| p.key.clone()));
        produced.extend(self.outside_inputs.iter().cloned());

        // Kahn's algorithm (level-by-level form). Each iteration pulls out
        // every node whose required inputs are already produced, assigns
        // it a NodeId, and pushes it into the current level. The level's
        // outputs then enter `produced` so the next iteration can advance.
        let mut remaining: Vec<Box<dyn PipelineNode>> = self.nodes;
        let mut levels: Vec<Vec<(NodeId, Box<dyn PipelineNode>)>> = Vec::new();
        let mut next_id: NodeId = 0;

        while !remaining.is_empty() {
            let (mut ready, still_waiting): (Vec<_>, Vec<_>) =
                remaining.into_iter().partition(|node| {
                    node.port_descriptor()
                        .required_inputs()
                        .all(|channel| produced.contains(channel))
                });

            remaining = still_waiting;

            // If no node is ready but `remaining` is non-empty, the sort
            // is stuck. Every input has a supplier (checked above), so the
            // stuck nodes are waiting on each other: a cycle.
            if ready.is_empty() {
                // Channels that *would* exist if the sort could continue.
                let mut pending_outputs: HashSet<ChannelKey> = HashSet::new();
                for node in &remaining {
                    for channel in node.port_descriptor().outputs() {
                        pending_outputs.insert(channel.clone());
                    }
                }

                // Cycle pass: a remaining node whose required inputs are
                // entirely covered by `produced ∪ pending_outputs` is
                // blocked purely by other stranded nodes — that is a
                // cycle. One Cycle error is emitted regardless of how
                // many nodes participate.
                let is_cycle_detected = remaining.iter().any(|node| {
                    node.port_descriptor().required_inputs().all(|channel| {
                        produced.contains(channel) || pending_outputs.contains(channel)
                    })
                });

                if is_cycle_detected {
                    let participants = remaining
                        .iter()
                        .filter(|node| {
                            node.port_descriptor().required_inputs().all(|channel| {
                                produced.contains(channel) || pending_outputs.contains(channel)
                            })
                        })
                        .map(|node| node.name().to_string())
                        .collect();
                    errors.push(PipelineBuildError::Cycle { participants });
                }

                break;
            }

            // Promote this level's outputs into `produced` so the next
            // iteration can advance.
            produced.extend(
                ready
                    .iter()
                    .flat_map(|node| node.port_descriptor().outputs().iter())
                    .cloned(),
            );

            // Sort by name so ids don't depend on the order nodes were added.
            // Assign NodeIds in level-major order as we go — this is the
            // same order the rate-timer array will be indexed by at tick
            // time, so the two stay in lockstep without a second pass.
            ready.sort_by(|a, b| a.name().cmp(b.name()));
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
        // A channel the body advertises via `capabilities.publishes` but
        // which no in-graph node consumes intentionally has no slot — the
        // host's `bus.write(...)` for such a channel returns
        // `ChannelError::UnknownChannel` and drops silently. The body
        // declares what it offers; the bus tracks intra-graph flow. The
        // two overlap iff a consumer exists.
        let descriptor_iter = levels
            .iter()
            .flat_map(|level| level.iter().map(|(_, node)| node.port_descriptor()));

        let mut bus = PortBus::new(descriptor_iter);

        // `command` is the one channel with an out-of-graph consumer: a
        // control-consuming body reads it back through `read_control`. Guarantee
        // its slot so a teleop-only stack (no controller, no in-graph producer)
        // can still have the host's command land instead of dropping.
        if self.capabilities.consumes_control {
            bus.ensure_slot(control::command::<BodyTwist>().into());
        }

        // One timer per node, indexed by NodeId (which matches level-major
        // iteration order below).
        let mut rate_timers: Vec<RateTimer> = Vec::with_capacity(next_id as usize);
        for level in &levels {
            for (_, node) in level {
                rate_timers.push(RateTimer::new(node.port_descriptor().rate()));
            }
        }

        log_resolved_dag(&levels, &self.capabilities, &self.outside_inputs);

        Ok(AutonomyPipeline {
            levels,
            bus,
            rate_timers,
            outside_inputs: self.outside_inputs,
        })
    }
}

/// One-shot startup dump of the resolved DAG. Emits at `info` so it's
/// visible with the default `helios=info` filter; nothing else in the
/// per-tick path emits at that level, so this stays a single block.
fn log_resolved_dag(
    levels: &[Vec<(NodeId, Box<dyn PipelineNode>)>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
) {
    info!(
        target: "helios_runtime::pipeline",
        levels = levels.len(),
        nodes = levels.iter().map(|level| level.len()).sum::<usize>(),
        "resolved autonomy pipeline",
    );

    // Body line: which body the graph runs against and what it offers. Indented
    // two spaces to nest under the pipeline header, matching the node lines below.
    info!(
        target: "helios_runtime::pipeline",
        name = capabilities.name,
        consumes_control = capabilities.consumes_control,
        published = capabilities.publishes.len(),
        "  body"
    );

    // One line per channel the body publishes, nested another level under body.
    for pc in &capabilities.publishes {
        info!(
            target: "helios_runtime::pipeline",
            channel = %format_key_short(&pc.key),
            provenance = ?pc.provenance,
            "    publishes"
        )
    }

    // Outside inputs: what an operator or mission system sends, kept apart from
    // the body's channels. Same nesting as the body section.
    info!(
        target: "helios_runtime::pipeline",
        declared = outside_inputs.len(),
        "  outside inputs"
    );

    for key in outside_inputs {
        info!(
            target: "helios_runtime::pipeline",
            channel = %format_key_short(key),
            "    input"
        )
    }

    for (level_idx, level) in levels.iter().enumerate() {
        for (node_id, node) in level {
            let descriptor = node.port_descriptor();
            let rate = match descriptor.rate() {
                Some(hz) => format!("{hz} Hz"),
                None => "every tick".to_string(),
            };
            let inputs = format_keys(descriptor.required_inputs());
            let optional = format_keys(descriptor.optional_inputs());
            let outputs = format_keys(descriptor.outputs());
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

fn format_keys<'a>(keys: impl IntoIterator<Item = &'a ChannelKey>) -> String {
    let joined = keys
        .into_iter()
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
    /// the only way values from outside the graph enter it: the host writes
    /// body measurements and operator or mission inputs here.
    bus: PortBus,
    /// Per-node rate gating, indexed by [`NodeId`]. A node with
    /// `rate: None` fires every tick.
    rate_timers: Vec<RateTimer>,
    /// Inputs declared as sent from outside the robot, kept from the build.
    outside_inputs: Vec<ChannelKey>,
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
