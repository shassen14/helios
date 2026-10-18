//! [`PipelineBuilder`]: collects one agent's nodes and node plans, the body's
//! capabilities and the outside inputs, then [`build`](PipelineBuilder::build)
//! checks them, resolves every channel's mount, builds the plans and
//! assembles the [`AutonomyPipeline`].

use super::{
    declaration::{NodeDeclaration, NodePlan},
    mounts::{check_mount_readings, resolve_mounts},
    ordering::order_into_levels,
    wiring::check_wiring,
};

use crate::{
    pipeline::autonomy_pipeline::{HostWrites, ScheduledNode},
    port::{ChannelDescription, PortBus, Writer},
    AutonomyPipeline, BodyCapabilities, ChannelKey, NodeId, PipelineBuildError, PipelineNode,
};

use helios_core::prelude::AgentId;
use helios_core::spatial::FrameId;

use std::collections::{HashMap, HashSet};

/// Constructs a [`AutonomyPipeline`] from a set of [`PipelineNode`]s and the
/// [`BodyCapabilities`] of the body the graph runs against.
///
/// Build sequence:
/// 0. Create the builder with [`new`](Self::new) for the agent the pipeline
///    serves. The body's mounts are checked against it.
/// 1. Register every node with [`add_node`](Self::add_node), or declare it
///    with [`add_plan`](Self::add_plan) to have it built once the graph
///    checks out.
/// 2. Declare the channels the body publishes — measurement channels, oracle
///    reference channels, health status — via
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
    agent: AgentId,
    nodes: Vec<NodeDeclaration>,
    capabilities: BodyCapabilities,
    outside_inputs: Vec<ChannelKey>,
    estimate_root: Option<FrameId>,
}

impl PipelineBuilder {
    /// Creates an empty builder for `agent`'s pipeline with a default (empty)
    /// [`BodyCapabilities`]. Add nodes and declare the body's published
    /// channels via [`with_body_capabilities`](Self::with_body_capabilities)
    /// before calling [`build`](Self::build).
    pub fn new(agent: AgentId) -> Self {
        PipelineBuilder {
            agent,
            nodes: vec![],
            capabilities: BodyCapabilities::default(),
            outside_inputs: vec![],
            estimate_root: None,
        }
    }

    /// Registers one node. Order does not matter — the topological sort
    /// places each node in the correct level based on its declared inputs
    /// and outputs.
    pub fn add_node(mut self, node: Box<dyn PipelineNode>) -> Self {
        self.nodes.push(node.into());
        self
    }

    /// Declares one node by its plan. The graph is checked and ordered on the
    /// plan's declared ports; [`build`](Self::build) builds the node only once
    /// both pass.
    pub fn add_plan(mut self, plan: NodePlan) -> Self {
        self.nodes.push(plan.into());
        self
    }

    /// Registers one node, built or planned.
    pub(crate) fn add_declaration(mut self, declaration: NodeDeclaration) -> Self {
        self.nodes.push(declaration);
        self
    }

    /// Declares the channels the body publishes (sensor signals, oracle
    /// reference channels, health). Each published channel counts as supplied in
    /// [`build`](Self::build), so consumers, required or optional, don't trip
    /// [`PipelineBuildError::UnsatisfiedInput`]; an unmet `Oracle`/`Health`
    /// input instead surfaces as
    /// [`PipelineBuildError::UnsatisfiedBodyCapabilities`].
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

    /// Names the frame the estimate holds the body's pose in, which the built
    /// pipeline exposes through [`AutonomyPipeline::estimate_root`]. Set by
    /// the estimate seam, which knows which estimator is authoritative.
    pub(crate) fn with_estimate_root(mut self, root: FrameId) -> Self {
        self.estimate_root = Some(root);
        self
    }

    /// Validates the registered nodes and builds a [`AutonomyPipeline`].
    ///
    /// Errors are checked in two stages, and every error within a stage is
    /// collected:
    /// 1. Wiring. Node names are unique and one level each, the body's unit
    ///    names are one level each, every mount the body states is a frame on
    ///    this agent's body other than `base_link`, every frame has one owner
    ///    (the body, or the one node that defines it), every output mounted at an input's
    ///    mount names a channel its node reads in the same tick, channel names
    ///    are well formed and each names one channel, each node's watchable
    ///    leaves are well formed, leaves and output channels sit outside the
    ///    pipeline's own group and apart, and every input, required or
    ///    optional, has exactly one supplier: a node output, a body channel,
    ///    or a declared outside input.
    ///    - [`PipelineBuildError::DuplicateNodeName`] — two nodes share a
    ///      name.
    ///    - [`PipelineBuildError::MalformedNodeName`] — a node name is not
    ///      one level.
    ///    - [`PipelineBuildError::MalformedUnitName`] — a unit the body
    ///      declares is not one level.
    ///    - [`PipelineBuildError::RefusedBodyMount`] — the body states a
    ///      mount off its own body, at `base_link`, or with a malformed leaf.
    ///    - [`PipelineBuildError::FrameDefinedTwice`] — two nodes define one
    ///      frame, or a node defines `base_link` or a frame the body mounts a
    ///      channel at.
    ///    - [`PipelineBuildError::MountFromUnreadInput`] — a node mounts an
    ///      output at the mount of a channel it does not read in the same
    ///      tick.
    ///    - [`PipelineBuildError::MalformedChannelName`] — a channel name is
    ///      not levels joined by `.`.
    ///    - [`PipelineBuildError::ChannelNameReused`] — one name names
    ///      channels of different kinds or types.
    ///    - [`PipelineBuildError::MalformedObservable`] — a leaf is not
    ///      levels joined by `.`.
    ///    - [`PipelineBuildError::ReservedObservable`] — a node declares a
    ///      leaf in the pipeline's own group.
    ///    - [`PipelineBuildError::DuplicateObservable`] — a node names a
    ///      watchable leaf twice.
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
    /// 3. Construction, run only when ordering succeeds. Each node output's
    ///    mount is resolved from its rule in level order, every mount a node
    ///    reads through the tf tree is checked to exist, then every plan is
    ///    built from its declared ports and its inputs' mounts, level by
    ///    level.
    ///    - [`PipelineBuildError::MountReadWithoutMount`] — a node reads the
    ///      mount of an input no writer states a mount for.
    ///    - [`PipelineBuildError::ConstructionFailed`] — a plan's constructor
    ///      failed.
    ///    - [`PipelineBuildError::ConstructedNodeMisnamed`] — a plan built a
    ///      node under another name.
    ///
    /// Within a level, nodes are sorted by name, so levels and [`NodeId`](crate::NodeId)s
    /// do not depend on the order nodes were added.
    ///
    /// On success, [`NodeId`](crate::NodeId)s are assigned in level-major order starting
    /// at `0`, indexing into the pipeline's rate-timer array.
    pub fn build(self) -> Result<AutonomyPipeline, Vec<PipelineBuildError>> {
        let descriptions = self.describe_channels();
        let errors = check_wiring(&self.agent, &self.nodes, &descriptions);

        // Stop before ordering: a node downstream of a missing input would
        // otherwise be reported as a cycle member too.
        if !errors.is_empty() {
            return Err(errors);
        }

        // Seed `produced` with channels written from outside the graph, so
        // the sort treats them as already written.
        let produced: HashSet<ChannelKey> = descriptions
            .iter()
            .filter(|description| !matches!(description.writer(), Writer::Node { .. }))
            .map(|description| description.key().clone())
            .collect();

        let levels = order_into_levels(self.nodes, produced)?;
        let descriptions = resolve_mounts(&levels, descriptions);
        let mut errors = vec![];
        check_mount_readings(&levels, &descriptions, &mut errors);
        if !errors.is_empty() {
            return Err(errors);
        }

        // The bus holds every channel a writer declares, read or not, so a
        // host's write to a channel its body declared always lands and a
        // write to any other channel is `ChannelError::UnknownChannel`.
        let bus = PortBus::new(&descriptions);

        let levels = construct_levels(levels, &descriptions, &self.agent)?;
        let host_writes = HostWrites::new(&self.agent, &descriptions);

        Ok(AutonomyPipeline {
            agent: self.agent,
            levels,
            bus,
            host_writes,
            descriptions,
            estimate_root: self.estimate_root,
        })
    }

    /// Every channel some writer declares, described with the writer whose
    /// declaration it came in: the body's, then the outside inputs, then each
    /// node's outputs in the order nodes were added. A channel two writers
    /// declare appears twice; the wiring check refuses it unless it is keyed
    /// and their instances differ.
    fn describe_channels(&self) -> Vec<ChannelDescription> {
        let body = self
            .capabilities
            .publishes
            .iter()
            .map(ChannelDescription::body);
        let outside = self
            .outside_inputs
            .iter()
            .map(|key| ChannelDescription::outside(key.clone()));
        let nodes = self.nodes.iter().flat_map(|node| {
            node.port_descriptor().output_ports().iter().map(|port| {
                ChannelDescription::node(port.channel().clone(), node.name())
                    .with_instances(port.instances().to_vec())
            })
        });
        body.chain(outside).chain(nodes).collect()
    }
}

/// Builds every planned node, level by level, with the mounts of its inputs
/// from `descriptions`, and gives each node the state the pipeline keeps for
/// it as a node of `agent`'s pipeline, keeping the levels and the order
/// within them. Every failure is collected.
fn construct_levels(
    levels: Vec<Vec<(NodeId, NodeDeclaration)>>,
    descriptions: &[ChannelDescription],
    agent: &AgentId,
) -> Result<Vec<Vec<ScheduledNode>>, Vec<PipelineBuildError>> {
    let descriptions: HashMap<&ChannelKey, &ChannelDescription> = descriptions
        .iter()
        .map(|description| (description.key(), description))
        .collect();
    let mut errors = vec![];
    let mut constructed = Vec::with_capacity(levels.len());
    for level in levels {
        let mut scheduled = Vec::with_capacity(level.len());
        for (node_id, declaration) in level {
            match declaration.construct(&descriptions) {
                Ok(node) => scheduled.push(ScheduledNode::new(node_id, node, agent)),
                Err(error) => errors.push(error),
            }
        }
        constructed.push(scheduled);
    }

    if errors.is_empty() {
        Ok(constructed)
    } else {
        Err(errors)
    }
}
