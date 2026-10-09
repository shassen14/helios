//! The stack-to-pipeline stages behind [`build_pipeline`].
//!
//! [`build_pipeline`] is the single entry point: given a fully-resolved
//! [`AutonomyStackConfig`] and an [`AutonomyRegistry`], it constructs every
//! [`PipelineNode`](crate::pipeline::node::PipelineNode), declares sensor signal channels, validates the graph, and
//! returns a ready-to-tick [`AutonomyPipeline`].
//!
//! ## What the host provides
//!
//! Three things cannot come from config — they are host-specific runtime tokens:
//!
//! - `agent` — the agent's stable [`AgentId`], the config name every subsystem
//!   uses to key this agent's frames. All spine frames (`base_link`, `odom`) and
//!   sensor frames are built from it, so the same identity is valid in sim and
//!   on hardware.
//! - `sensor_channels` — the set of sensor channel names the host actually
//!   publishes for this agent. Each channel name is also the leaf of the
//!   sensor's [`FrameId`](helios_core::spatial::FrameId) (`FrameId::sensor(agent, channel_name)`), so a node
//!   that reads a channel the host does not provide, and no node derives, is
//!   rejected at build time rather than silently never running.
//! - `host_capabilities` — the body as the host describes it: its name, its
//!   actuators (which the actuator seam checks every driven actuator against),
//!   and the body channels that are not config-derived sensors (today:
//!   `oracle/*` reference channels; later: `health/*`). The
//!   assembler appends config-derived sensor channels onto
//!   `host_capabilities.publishes` before handing the merged value to
//!   [`PipelineBuilder::with_body_capabilities`]. Only host-published sensor
//!   inputs are appended; an internal input, or a sensor channel a
//!   node derives, must be produced inside the graph, so a
//!   missing producer still fails the build.
//!
//! Inputs an operator or mission system sends — each planner's goal, and the
//! teleop intent when teleop is wired — are not body channels. They are
//! declared separately through [`PipelineBuilder::with_outside_inputs`]: the
//! `[nodes]` factory of each node that reads one declares it.
//!
//! Everything else — algorithm kinds, noise params, physical constants,
//! channel names — comes from `stack`.

use super::config::AutonomyStackConfig;
use super::error::PipelineAssemblyError;
use super::instantiate::instantiate;
use super::registry::AutonomyRegistry;
use super::seams::actuators::actuator_merge;
use super::seams::command::command_sums;
use super::seams::estimate::estimate_relay;
use super::seams::reference::reference_selector;
use super::sensor_inputs::{derived_channels, SensorInputs};

use crate::body::{BodyCapabilities, Provenance, PublishedChannel};
use crate::pipeline::dag_log::log_resolved_dag;
use crate::pipeline::AutonomyPipeline;
use crate::pipeline::PipelineBuilder;
use crate::port::ChannelKey;

use helios_core::prelude::AgentId;

use std::collections::HashSet;

/// Builds a fully-validated [`AutonomyPipeline`] from a resolved [`AutonomyStackConfig`].
///
/// Each node's own config is checked by its kind's factory; the stack-level
/// checks (an unsourced sensor input, an unsatisfiable graph edge) follow once
/// every node is built.
///
/// # Parameters
///
/// - `stack` — fully-resolved autonomy config (no unresolved `from` refs).
/// - `registry` — factory registry, typically `AutonomyRegistry::default()`.
/// - `agent` — the agent's stable [`AgentId`] (its config name); every frame
///   this agent owns is keyed by it.
/// - `sensor_channels` — the set of sensor channel names the host publishes for
///   this agent. A node reading a sensor channel absent from this set, and
///   derived by no node, is an
///   [`UnpublishedSensorInput`](PipelineAssemblyError::UnpublishedSensorInput).
/// - `host_capabilities` — the body's capabilities as the host describes them
///   (name, actuators, reference channels such as `oracle/*`).
///   The assembler extends `host_capabilities.publishes` with the
///   config-derived sensor channels before building. Goals and teleop intent
///   are declared separately as outside inputs.
pub fn build_pipeline(
    stack: &AutonomyStackConfig,
    registry: &AutonomyRegistry,
    agent: AgentId,
    sensor_channels: &HashSet<String>,
    mut host_capabilities: BodyCapabilities,
) -> Result<AutonomyPipeline, Vec<PipelineAssemblyError>> {
    let mut errors: Vec<PipelineAssemblyError> = vec![];
    let mut builder = PipelineBuilder::new();
    // The body's sensor channels the stack reads. Merged into the body's
    // `publishes` before building, so the build counts them as supplied.
    let mut external_channels: Vec<ChannelKey> = vec![];
    // Inputs sent from outside the robot that the stack reads, such as each
    // planner's goal and the operator's teleop intent. Declared to the builder
    // apart from the body's channels.
    let mut outside_inputs: Vec<ChannelKey> = vec![];

    // --- Nodes ---
    // Every `[nodes]` entry (e.g. a range field flattened to a point cloud). A
    // sensor channel one of them writes is a derived channel, which consumers
    // read exactly as they read host channels. All of them are built before any
    // input is seeded, so the derived set is complete even when one node reads
    // another's output. A failure here returns at once: with the derived set
    // incomplete, every consumer of a missing channel would report a misleading
    // unpublished input.
    let instantiated = instantiate(stack, registry, &agent, sensor_channels)?;
    let derived = derived_channels(&instantiated.nodes, sensor_channels)?;
    outside_inputs.extend(instantiated.outside_inputs);

    let sensor_inputs = SensorInputs {
        host: sensor_channels,
        derived: &derived,
    };
    let nodes = instantiated.nodes;

    // --- Estimate seam ---
    // The `[estimate]` section names the estimator whose state the rest of
    // the stack reads; a relay forwards it and publishes the TF edge.
    match estimate_relay(stack.estimate.as_ref(), &nodes, &agent) {
        Ok(Some(relay)) => builder = builder.add_node(relay),
        Ok(None) => {}
        Err(seam_errors) => errors.extend(seam_errors),
    }

    // --- Reference seam ---
    // The `[reference]` section names the nodes whose references contend for
    // the one the controllers track. Members are `[nodes]` entries, so the seam
    // resolves them before the nodes move into the builder.
    match reference_selector(stack.reference.as_ref(), &nodes) {
        Ok(Some(selector)) => builder = builder.add_node(selector),
        Ok(None) => {}
        Err(seam_errors) => errors.extend(seam_errors),
    }

    // --- Command seam ---
    // Each `[command.<fold>]` table names the nodes whose commands one `Sum`
    // folds into a channel named after the fold, which an allocator reads.
    match command_sums(&stack.command, registry, &nodes) {
        Ok(sums) => {
            for sum in sums {
                builder = builder.add_node(sum);
            }
        }
        Err(seam_errors) => errors.extend(seam_errors),
    }

    // --- Actuator seam ---
    // The `[actuators]` section names the nodes whose partial actuator
    // commands one `Merge` unions into the command the body applies. What
    // each member drives is on its output port; the seam checks it against
    // the other members and against the body's actuators.
    match actuator_merge(stack.actuators.as_ref(), &host_capabilities, &nodes) {
        Ok(Some(merge)) => builder = builder.add_node(merge),
        Ok(None) => {}
        Err(seam_errors) => errors.extend(seam_errors),
    }

    for node in nodes {
        sensor_inputs.seed(node.as_ref(), &mut external_channels, &mut errors);
        builder = builder.add_node(node);
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    // Deduplicate both lists before handing them to the builder, keeping
    // insertion order so the resolved-config dump is stable. Two planners on one
    // goal channel share one outside input.
    let mut seen = HashSet::new();
    external_channels.retain(|key| seen.insert(key.clone()));
    let mut seen_outside = HashSet::new();
    outside_inputs.retain(|key| seen_outside.insert(key.clone()));

    host_capabilities
        .publishes
        .extend(external_channels.into_iter().map(|key| PublishedChannel {
            key,
            provenance: Provenance::Exact,
        }));

    // Kept for the startup log, which shows the body the graph was built
    // against.
    let capabilities = host_capabilities.clone();
    let pipeline = builder
        .with_body_capabilities(host_capabilities)
        .with_outside_inputs(outside_inputs)
        .build()
        .map_err(|build_errors| vec![PipelineAssemblyError::PipelineBuild(build_errors)])?;

    // Each `[nodes]` entry is labelled with its kind; a seam's node has no
    // entry and shows its name alone.
    log_resolved_dag(&pipeline, &capabilities, |name| stack.node_kind(name));
    Ok(pipeline)
}
