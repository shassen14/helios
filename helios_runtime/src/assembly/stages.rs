//! The stack-to-pipeline stages behind [`build_pipeline`].
//!
//! [`build_pipeline`] is the single entry point: given a fully-resolved
//! [`AutonomyStackConfig`] and an [`AutonomyRegistry`], it constructs every
//! [`PipelineNode`](crate::pipeline::node::PipelineNode), validates the graph, and
//! returns a ready-to-tick [`AutonomyPipeline`].
//!
//! ## What the host provides
//!
//! Two things cannot come from config — they are host-specific runtime tokens:
//!
//! - `agent` — the agent's stable [`AgentId`], the config name every subsystem
//!   uses to key this agent's frames. It must be non-empty and free of `.`,
//!   since it is one part of every watcher path for the agent. All spine frames (`base_link`, `odom`) and
//!   sensor frames are built from it, so the same identity is valid in sim and
//!   on hardware.
//! - `body` — the body as the host describes it: its actuators (which the
//!   actuator seam checks every driven actuator against) and every channel
//!   it publishes, each typed and with the unit that produces it: sensor
//!   channels, oracle reference channels, later health channels. It goes to
//!   [`PipelineBuilder::with_body_capabilities`] as given, so a node reading a
//!   sensor channel the body does not publish, and no node writes, fails the
//!   build rather than silently never running.
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

use crate::body::BodyCapabilities;
use crate::pipeline::dag_log::log_resolved_dag;
use crate::pipeline::AutonomyPipeline;
use crate::pipeline::PipelineBuilder;
use crate::port::ChannelKey;

use helios_core::{kernel::identifier::is_level, prelude::AgentId};

use std::collections::HashSet;

/// Builds a fully-validated [`AutonomyPipeline`] from a resolved [`AutonomyStackConfig`].
///
/// Each node's own config is checked by its kind's factory; the stack-level
/// checks (an input nothing supplies, a name naming two channels) follow once
/// every node is built.
///
/// # Parameters
///
/// - `stack` — fully-resolved autonomy config (no unresolved `from` refs).
/// - `registry` — factory registry, typically `AutonomyRegistry::default()`.
/// - `agent` — the agent's stable [`AgentId`] (its config name); every frame
///   this agent owns is keyed by it. An empty or dotted name is a
///   [`MalformedAgentName`](PipelineAssemblyError::MalformedAgentName), and
///   nothing else is checked.
/// - `body` — the body as the host describes it: its actuators and every
///   channel it publishes. Goals and teleop intent are not the body's; each
///   node's factory declares the ones it reads as outside inputs.
pub fn build_pipeline(
    stack: &AutonomyStackConfig,
    registry: &AutonomyRegistry,
    agent: AgentId,
    body: BodyCapabilities,
) -> Result<AutonomyPipeline, Vec<PipelineAssemblyError>> {
    // Checked first and alone: every frame and watcher path for this agent
    // is built from its name, so anything checked after would only repeat it.
    if !is_level(agent.as_str()) {
        return Err(vec![PipelineAssemblyError::MalformedAgentName {
            agent: agent.to_string(),
        }]);
    }

    let mut errors: Vec<PipelineAssemblyError> = vec![];
    let mut builder = PipelineBuilder::new();
    // Inputs sent from outside the robot that the stack reads, such as each
    // planner's goal and the operator's teleop intent. Declared to the builder
    // apart from the body's channels.
    let mut outside_inputs: Vec<ChannelKey> = vec![];

    // --- Nodes ---
    // Every `[nodes]` entry. A failure here returns at once: with a node
    // missing, every reader of its outputs would report a misleading
    // unsupplied input.
    let instantiated = instantiate(stack, registry, &agent)?;
    outside_inputs.extend(instantiated.outside_inputs);
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
    match actuator_merge(stack.actuators.as_ref(), &body, &nodes) {
        Ok(Some(merge)) => builder = builder.add_node(merge),
        Ok(None) => {}
        Err(seam_errors) => errors.extend(seam_errors),
    }

    for node in nodes {
        builder = builder.add_node(node);
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    // Deduplicate before handing the list to the builder, keeping insertion
    // order so the resolved-config dump is stable. Two planners on one goal
    // channel share one outside input.
    let mut seen_outside = HashSet::new();
    outside_inputs.retain(|key| seen_outside.insert(key.clone()));

    let pipeline = builder
        .with_body_capabilities(body)
        .with_outside_inputs(outside_inputs)
        .build()
        .map_err(|build_errors| vec![PipelineAssemblyError::PipelineBuild(build_errors)])?;

    // Each `[nodes]` entry is labelled with its kind; a seam's node has no
    // entry and shows its name alone.
    log_resolved_dag(&pipeline, &agent, |name| stack.node_kind(name));
    Ok(pipeline)
}
