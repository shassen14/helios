//! The stack-to-pipeline stages behind [`build_pipeline`].
//!
//! [`build_pipeline`] is the single entry point: given a fully-resolved
//! [`AutonomyStack`] and an [`AutonomyRegistry`], it constructs every
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
//!   sensor's [`FrameId`](helios_core::spatial::FrameId) (`FrameId::sensor(agent, channel_name)`), so an aiding
//!   or augmentation entry that names a channel the host does not provide is
//!   rejected at build time rather than silently failing to resolve at tick time.
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
//!
//! ## Sensor payload dispatch
//!
//! Aiding handler construction requires a concrete `T: SensorPayload` at
//! compile time. The assembler matches the `sensor_payload` string from
//! [`AidingConfig`](crate::config::AidingConfig) to one of the known implementors via an inline `match`.
//! This list must stay in sync with `KNOWN_SENSOR_PAYLOADS` in `validation.rs`
//! and with the `SensorPayload` impls in `helios_core::interchange::measurement::sensor`.
//!
//! If third-party sensor payload types become a real requirement, this can be
//! promoted to a registry family (`register_aiding_handler_factory`). For the
//! current set of five built-in types, the inline match is sufficient.

use super::contexts::MockEstimatorBuildContext;
use super::error::PipelineAssemblyError;
use super::instantiate::instantiate;
use super::registry::AutonomyRegistry;
use super::seams::actuators::actuator_merge;
use super::seams::command::command_sums;
use super::seams::reference::reference_selector;
use super::sensor_inputs::{derived_channels, SensorInputs};

use crate::body::{BodyCapabilities, Provenance, PublishedChannel};
use crate::config::AutonomyStack;
use crate::config::{EstimatorConfig, MOCK_ORACLE_KIND, UKF_KIND};
use crate::nodes::gaussian_estimator;
use crate::pipeline::AutonomyPipeline;
use crate::pipeline::PipelineBuilder;
use crate::port::ChannelKey;

use helios_core::prelude::AgentId;

use std::collections::HashSet;

/// Builds a fully-validated [`AutonomyPipeline`] from a resolved [`AutonomyStack`].
///
/// Runs [`crate::validation::validate_autonomy_config`] against the registry's
/// capabilities first and short-circuits with
/// [`PipelineAssemblyError::InvalidConfig`] if the config is invalid, so every
/// host gets the same static checks with legible messages before assembly is
/// attempted. Errors that need host-supplied context (an aiding channel with no
/// unknown sensor channel, an unsatisfiable graph edge) still surface from assembly.
///
/// # Parameters
///
/// - `stack` — fully-resolved autonomy config (no unresolved `from` refs).
/// - `registry` — factory registry, typically `AutonomyRegistry::default()`.
/// - `agent` — the agent's stable [`AgentId`] (its config name); every frame
///   this agent owns is keyed by it.
/// - `sensor_channels` — the set of sensor channel names the host publishes for
///   this agent. An aiding or augmentation entry naming a channel absent from
///   this set is an [`UnknownSensorChannel`](PipelineAssemblyError::UnknownSensorChannel).
/// - `host_capabilities` — the body's capabilities as the host describes them
///   (name, actuators, reference channels such as `oracle/*`).
///   The assembler extends `host_capabilities.publishes` with the
///   config-derived sensor channels before building. Goals and teleop intent
///   are declared separately as outside inputs.
pub fn build_pipeline(
    stack: &AutonomyStack,
    registry: &AutonomyRegistry,
    agent: AgentId,
    sensor_channels: &HashSet<String>,
    mut host_capabilities: BodyCapabilities,
) -> Result<AutonomyPipeline, Vec<PipelineAssemblyError>> {
    // Static validation runs before any node is built: a config-level mistake
    // (unknown kind, an augmentation no aiding source observes) is reported as
    // itself rather than as a downstream factory or unsatisfied-input failure.
    let config_errors =
        crate::validation::validate_autonomy_config(stack, &registry.capabilities());
    if !config_errors.is_empty() {
        return Err(vec![PipelineAssemblyError::InvalidConfig(config_errors)]);
    }

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

    // --- Reference seam ---
    // The `[reference]` section names the nodes whose references contend for
    // the one the controllers track. Members are `[nodes]` entries, so the seam
    // resolves them before the nodes move into the builder.
    match reference_selector(stack.reference.as_ref(), &instantiated.nodes) {
        Ok(Some(selector)) => builder = builder.add_node(selector),
        Ok(None) => {}
        Err(seam_errors) => errors.extend(seam_errors),
    }

    // --- Command seam ---
    // Each `[command.<fold>]` table names the nodes whose commands one `Sum`
    // folds into a channel named after the fold, which an allocator reads.
    match command_sums(&stack.command, registry, &instantiated.nodes) {
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
    match actuator_merge(
        stack.actuators.as_ref(),
        &host_capabilities,
        &instantiated.nodes,
    ) {
        Ok(Some(merge)) => builder = builder.add_node(merge),
        Ok(None) => {}
        Err(seam_errors) => errors.extend(seam_errors),
    }

    for node in instantiated.nodes {
        sensor_inputs.seed(node.as_ref(), &mut external_channels, &mut errors);
        builder = builder.add_node(node);
    }

    // --- Estimators ---
    for (instance_name, est_cfg) in &stack.estimators {
        match build_estimator_node(instance_name, est_cfg, &agent, sensor_channels, registry) {
            Ok(node) => {
                sensor_inputs.seed(node.as_ref(), &mut external_channels, &mut errors);
                builder = builder.add_node(node);
            }
            Err(e) => errors.push(e),
        }
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

    builder
        .with_body_capabilities(host_capabilities)
        .with_outside_inputs(outside_inputs)
        .build()
        .map_err(|build_errors| vec![PipelineAssemblyError::PipelineBuild(build_errors)])
}

fn build_estimator_node(
    instance_name: &str,
    est_cfg: &EstimatorConfig,
    agent: &AgentId,
    sensor_channels: &HashSet<String>,
    registry: &AutonomyRegistry,
) -> Result<Box<dyn crate::pipeline::node::PipelineNode>, PipelineAssemblyError> {
    // Dispatch on estimator family. Each family owns a different build
    // context shape (Gaussian needs aiding handlers; mock needs none;
    // particle will need particle-count / resampling). Adding a new
    // family means a new factory map in the registry, a new context in
    // `contexts.rs`, and a new arm here.
    match est_cfg {
        EstimatorConfig::Ekf(ekf_cfg) => gaussian_estimator::assemble(
            instance_name,
            est_cfg,
            ekf_cfg,
            agent,
            sensor_channels,
            registry,
        ),
        EstimatorConfig::Ukf(_) => Err(PipelineAssemblyError::FactoryFailure {
            node_kind: UKF_KIND.to_string(),
            reason: "UKF not yet implemented".to_string(),
        }),
        EstimatorConfig::MockOracle(_) => {
            // Mocks declare oracle inputs through their port descriptor and
            // the build-time check against BodyCapabilities decides whether
            // the body satisfies them.
            registry
                .build_mock_estimator(
                    MOCK_ORACLE_KIND,
                    est_cfg.clone(),
                    MockEstimatorBuildContext {
                        agent: agent.clone(),
                        instance_name: instance_name.to_string(),
                    },
                )
                .map_err(|reason| PipelineAssemblyError::FactoryFailure {
                    node_kind: MOCK_ORACLE_KIND.to_string(),
                    reason,
                })
        }
    }
}
