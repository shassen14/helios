//! [`PipelineBuilder`]: collects the nodes, the body's capabilities and the
//! outside inputs, then [`build`](PipelineBuilder::build) checks them and
//! assembles the [`AutonomyPipeline`].

use super::{ordering::order_into_levels, wiring::check_wiring};

use crate::{
    pipeline::autonomy_pipeline::ScheduledNode, port::PortBus, AutonomyPipeline, BodyCapabilities,
    ChannelKey, PipelineBuildError, PipelineNode,
};

use std::collections::HashSet;

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
/// last-seen [`Stamped::timestamp`](crate::Stamped::timestamp) internally.
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
    /// Creates an empty builder with a default (empty)
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

    /// Declares the channels the body publishes (sensor signals, oracle
    /// reference channels, health). Each published channel counts as supplied in
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
    /// 1. Wiring. Node names are unique, each node's watchable leaves are
    ///    well formed, outside the pipeline's own group, distinct and clear
    ///    of its output channels' paths, and every input,
    ///    required or optional, has exactly one supplier: a node output, a
    ///    body channel, or a declared outside input.
    ///    - [`PipelineBuildError::DuplicateNodeName`] — two nodes share a
    ///      name.
    ///    - [`PipelineBuildError::MalformedObservable`] — a leaf has an
    ///      empty part or a `/`.
    ///    - [`PipelineBuildError::ReservedObservable`] — a node declares a
    ///      leaf in the pipeline's own group.
    ///    - [`PipelineBuildError::DuplicateObservable`] — a node names a
    ///      watchable leaf twice.
    ///    - [`PipelineBuildError::ObservableCollidesWithOutput`] — a leaf has
    ///      the same path as one of the node's output channels.
    ///    - [`PipelineBuildError::MultipleSuppliers`] — two suppliers
    ///      provide the same channel.
    ///    - [`PipelineBuildError::UnsatisfiedInput`] /
    ///      [`PipelineBuildError::UnsatisfiedBodyCapabilities`] — an input
    ///      has no supplier.
    /// 2. Ordering, run only when wiring is clean, so a missing input never
    ///    also shows up as a cycle.
    ///    - [`PipelineBuildError::Cycle`] — nodes wait on each other's
    ///      outputs in the same tick; one error per loop.
    ///    - [`PipelineBuildError::StuckWithoutCycle`] — the sort stopped but
    ///      found no loop, which means a bug in the build.
    ///
    /// Within a level, nodes are sorted by name, so levels and [`NodeId`](crate::NodeId)s
    /// do not depend on the order nodes were added.
    ///
    /// On success, [`NodeId`](crate::NodeId)s are assigned in level-major order starting
    /// at `0`, indexing into the pipeline's rate-timer array.
    pub fn build(self) -> Result<AutonomyPipeline, Vec<PipelineBuildError>> {
        let errors = check_wiring(&self.nodes, &self.capabilities, &self.outside_inputs);

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

        let levels = order_into_levels(self.nodes, produced)?;

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

        let bus = PortBus::new(descriptor_iter);

        // Each node gets the state the pipeline keeps for it, keeping the
        // levels and the order within them.
        let levels = levels
            .into_iter()
            .map(|level| {
                level
                    .into_iter()
                    .map(|(node_id, node)| ScheduledNode::new(node_id, node))
                    .collect()
            })
            .collect();

        Ok(AutonomyPipeline {
            levels,
            bus,
            outside_inputs: self.outside_inputs,
        })
    }
}
