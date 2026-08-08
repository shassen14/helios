//! Config-driven pipeline assembler.
//!
//! [`build_pipeline`] is the single entry point: given a fully-resolved
//! [`AutonomyStack`] and an [`AutonomyRegistry`], it constructs every
//! [`PipelineNode`], declares sensor signal channels, validates the graph, and
//! returns a ready-to-tick [`AutonomyPipeline`].
//!
//! ## What the host provides
//!
//! Three things cannot come from config — they are host-specific runtime tokens:
//!
//! - `agent_handle` — the agent's [`FrameHandle`], assigned by the host's
//!   entity system (Bevy `Entity` bits in sim, static calibration ID on hw).
//! - `sensor_frame_handles` — maps each aiding `input_channel` string to the
//!   [`FrameHandle`] of the physical sensor that publishes on it. Used to build
//!   the `MeasurementModelBuildContext` for each aiding handler.
//! - `host_capabilities` — the body's name, whether it consumes control, and
//!   the channels the host publishes outside the autonomy stack (today:
//!   `oracle/*`; later: `health/*`). The assembler appends config-derived
//!   sensor channels onto `host_capabilities.publishes` before handing the
//!   merged value to [`PipelineBuilder::with_body_capabilities`].
//!
//! Everything else — algorithm kinds, noise params, physical constants,
//! channel names — comes from `stack`.
//!
//! ## Sensor payload dispatch
//!
//! Aiding handler construction requires a concrete `T: SensorPayload` at
//! compile time. The assembler matches the `sensor_payload` string from
//! [`AidingConfig`] to one of the known implementors via an inline `match`.
//! This list must stay in sync with `KNOWN_SENSOR_PAYLOADS` in `validation.rs`
//! and with the `SensorPayload` impls in `helios_core::data::sensor`.
//!
//! If third-party sensor payload types become a real requirement, this can be
//! promoted to a registry family (`register_aiding_handler_factory`). For the
//! current set of five built-in types, the inline match is sufficient.

mod command;
mod error;

pub use self::error::PipelineAssemblyError;

use self::command::{
    resolve_command_topology, selector_policy, source_channel, CommandTopology,
    COMMAND_ARBITER_NODE, TELEOP_MAPPER_NODE,
};
use crate::body::{BodyCapabilities, Provenance, PublishedChannel};
use crate::channels::control;
use crate::config::TeleopMapperConfig;
use crate::config::{AutonomyStack, CommandSource};
use crate::config::{EstimatorConfig, MapLayerConfig};
use crate::nodes::combinators::Selector;
use crate::nodes::gaussian_estimator;
use crate::nodes::path_follower;
use crate::nodes::teleop::{TwistScale, TwistTeleopNode};
use crate::pipeline::autonomy_pipeline::PipelineBuilder;
use crate::pipeline::AutonomyPipeline;
use crate::port::{ChannelKey, InternalChannel};
use crate::registry::contexts::{
    AllocatorBuildContext, ControllerBuildContext, MapperBuildContext, MockEstimatorBuildContext,
    PathFollowerBuildContext, SearchPlannerBuildContext,
};
use crate::registry::AutonomyRegistry;

use helios_core::control::commands::{BodyTwist, TwistIntent};
use helios_core::data::primitives::FrameHandle;
use helios_core::frames::FrameAwareState;
use helios_core::mapping::MapData;
use helios_core::planning::types::Path;

use std::collections::{HashMap, HashSet};

/// Builds a fully-validated [`AutonomyPipeline`] from a resolved [`AutonomyStack`].
///
/// Runs [`crate::validation::validate_autonomy_config`] against the registry's
/// capabilities first and short-circuits with
/// [`PipelineAssemblyError::InvalidConfig`] if the config is invalid, so every
/// host gets the same static checks with legible messages before assembly is
/// attempted. Errors that need host-supplied context (an aiding channel with no
/// `FrameHandle`, an unsatisfiable graph edge) still surface from assembly.
///
/// # Parameters
///
/// - `stack` — fully-resolved autonomy config (no unresolved `from` refs).
/// - `registry` — factory registry, typically `AutonomyRegistry::default()`.
/// - `agent_handle` — host-assigned identity token for this agent.
/// - `sensor_frame_handles` — maps each aiding `input_channel` to the
///   [`FrameHandle`] of the physical sensor publishing on that channel.
///   Channels not used by any aiding entry may be absent.
/// - `host_capabilities` — host-supplied body capabilities (name,
///   `consumes_control`, host-published channels such as `oracle/*`).
///   The assembler extends `host_capabilities.publishes` with the
///   config-derived sensor channels before building.
pub fn build_pipeline(
    stack: &AutonomyStack,
    registry: &AutonomyRegistry,
    agent_handle: FrameHandle,
    sensor_frame_handles: &HashMap<String, FrameHandle>,
    mut host_capabilities: BodyCapabilities,
) -> Result<AutonomyPipeline, Vec<PipelineAssemblyError>> {
    // Static validation runs before any node is built: a config-level mistake
    // (unknown kind, planner reading an absent map layer) is reported as itself
    // rather than as a downstream factory or unsatisfied-input failure.
    let config_errors =
        crate::validation::validate_autonomy_config(stack, &registry.capabilities());
    if !config_errors.is_empty() {
        return Err(vec![PipelineAssemblyError::InvalidConfig(config_errors)]);
    }

    let mut errors: Vec<PipelineAssemblyError> = vec![];
    let mut builder = PipelineBuilder::new();
    // Channels supplied from outside the graph (sensor publishers, mission
    // layer, operator UI). Used to seed the topological sort so consumers
    // don't trip UnsatisfiedInput. Control channels are seeded per the
    // resolved command topology below, not unconditionally.
    let mut external_channels: Vec<ChannelKey> = vec![];

    // --- Estimators ---
    for (instance_name, est_cfg) in &stack.estimators {
        match build_estimator_node(
            instance_name,
            est_cfg,
            agent_handle,
            sensor_frame_handles,
            registry,
            &mut external_channels,
        ) {
            Ok(node) => {
                builder = builder.add_node(node);
            }
            Err(e) => errors.push(e),
        }
    }

    // --- Map layers ---
    for (map_name, map_cfg) in &stack.map_layers {
        if matches!(map_cfg, MapLayerConfig::None) {
            continue;
        }

        match registry.build_mapper(
            map_cfg.get_kind_str(),
            MapperBuildContext {
                agent_handle,
                instance_name: map_name.clone(),
                config: map_cfg.clone(),
            },
        ) {
            Ok(node) => {
                // FrameAwareState is produced by the estimator upstream;
                // every other required input of a mapper is an external
                // sensor channel that must seed the topological sort.
                let state_key: ChannelKey = InternalChannel::of::<FrameAwareState>().into();
                for key in &node.port_descriptor().required_inputs {
                    if *key != state_key {
                        external_channels.push(key.clone());
                    }
                }
                builder = builder.add_node(node);
            }
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: map_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    // --- Planners ---
    for (planner_name, plan_cfg) in &stack.search_planners {
        let level = plan_cfg.get_level_str();
        let map_channel = InternalChannel::named::<MapData>(level);
        let path_channel = InternalChannel::named::<Path>(planner_name.as_str());

        match registry.build_search_planner(
            plan_cfg.get_kind_str(),
            SearchPlannerBuildContext {
                agent_handle,
                instance_name: planner_name.clone(),
                config: plan_cfg.clone(),
                map_channel,
                path_channel,
            },
        ) {
            Ok(node) => {
                builder = builder.add_node(node);
            }
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: plan_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    // --- Path follower ---
    if let Some(pf_cfg) = &stack.path_following {
        match path_follower::resolve_path_channel(stack) {
            Ok(path_channel) => {
                match registry.build_path_follower(
                    pf_cfg.get_kind_str(),
                    PathFollowerBuildContext {
                        agent_handle,
                        config: pf_cfg.clone(),
                        path_channel,
                    },
                ) {
                    Ok(node) => {
                        builder = builder.add_node(node);
                    }
                    Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                        node_kind: pf_cfg.get_kind_str().to_string(),
                        reason,
                    }),
                }
            }
            Err(e) => errors.push(e),
        }
    }

    // --- Controllers ---
    // Resolve how the command terminal is fed before building controllers: the
    // controller's output channel depends on it. A lone autonomy source lets the
    // controller write `command` directly; otherwise it writes `autonomy` and
    // arbitration (or nothing) forwards from there.
    let command_topology =
        resolve_command_topology(&stack.command_arbitration, !stack.controllers.is_empty());
    let controller_output = match &command_topology {
        CommandTopology::Direct(CommandSource::Autonomy) => control::command::<BodyTwist>(),
        _ => control::autonomy::<BodyTwist>(),
    };

    for (controller_name, ctrl_cfg) in &stack.controllers {
        match registry.build_controller(
            ctrl_cfg.get_kind_str(),
            ControllerBuildContext {
                agent_handle,
                instance_name: controller_name.clone(),
                config: ctrl_cfg.clone(),
                output_channel: controller_output.clone(),
            },
        ) {
            Ok(node) => {
                builder = builder.add_node(node);
            }
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: ctrl_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    // Wire the command terminal (`command`, the channel `read_control` reads)
    // per the resolved topology. `BodyTwist` is fixed until the actuator seam.
    match command_topology {
        // Pure estimator / mapper agent: nothing produces commands.
        CommandTopology::None => {}

        // A lone internal source (autonomy) already writes `command` through its
        // retargeted producer (see `controller_output`), so there is nothing to
        // synthesize or seed. A lone host source never reaches this arm — it
        // resolves to `Arbitrated` and is relayed below.
        CommandTopology::Direct(_) => {}

        // Two or more sources contend: a `Selector` forwards the winner to
        // `command`. The higher-priority sources are freshness-gated `preferred`
        // inputs; the lowest is the always-available `base` fallback.
        CommandTopology::Arbitrated { preferred, base } => {
            let selector = Selector::<BodyTwist>::new(
                COMMAND_ARBITER_NODE,
                preferred.iter().map(|s| source_channel(*s)).collect(),
                source_channel(base),
                control::command::<BodyTwist>(),
                selector_policy(&stack.command_arbitration),
            );
            builder = builder.add_node(Box::new(selector));
        }
    }

    if stack
        .command_arbitration
        .sources
        .contains(&CommandSource::Teleop)
    {
        match &stack.teleop {
            Some(TeleopMapperConfig::Twist {
                surge,
                sway,
                heave,
                roll,
                pitch,
                yaw,
            }) => {
                let node = TwistTeleopNode::new(
                    TELEOP_MAPPER_NODE,
                    control::intent::<TwistIntent>(),
                    control::teleop::<BodyTwist>(),
                    TwistScale {
                        surge: *surge,
                        sway: *sway,
                        heave: *heave,
                        roll: *roll,
                        pitch: *pitch,
                        yaw: *yaw,
                    },
                );

                builder = builder.add_node(Box::new(node));

                external_channels.push(control::intent::<TwistIntent>().into());
            }
            None => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: "TeleopMapper".to_string(),
                reason: "teleop is a declared command source but no
  [teleop] mapper config was provided"
                    .to_string(),
            }),
        }
    }

    // --- Allocators ---
    // The allocator is a *consumer* of `command` (unlike the controllers /
    // arbiter above, which produce it) and produces the `actuators` terminal.
    // Both channels are graph-internal, so nothing is seeded onto
    // `external_channels`.
    for (allocator_name, alloc_cfg) in &stack.allocators {
        // Input is `command` at `BodyTwist` because `KinematicAckermannAllocator`
        // consumes `BodyTwist`. A non-`BodyTwist` allocator will source its input
        // type from config; that generalization waits for the first such impl.
        match registry.build_allocator(
            alloc_cfg.get_kind_str(),
            AllocatorBuildContext {
                agent_handle,
                instance_name: allocator_name.clone(),
                config: alloc_cfg.clone(),
                input_channel: control::command::<BodyTwist>(),
                output_channel: control::actuators(),
            },
        ) {
            Ok(node) => builder = builder.add_node(node),
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: alloc_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    // Deduplicate external channels before handing to the builder, keeping
    // insertion order so the resolved-config dump is stable.
    let mut seen = HashSet::new();
    external_channels.retain(|key| seen.insert(key.clone()));

    host_capabilities
        .publishes
        .extend(external_channels.into_iter().map(|key| PublishedChannel {
            key,
            provenance: Provenance::Exact,
        }));

    builder
        .with_body_capabilities(host_capabilities)
        .build()
        .map_err(|build_errors| vec![PipelineAssemblyError::PipelineBuild(build_errors)])
}

// --- Internals ---

fn build_estimator_node(
    instance_name: &str,
    est_cfg: &EstimatorConfig,
    agent_handle: FrameHandle,
    sensor_frame_handles: &HashMap<String, FrameHandle>,
    registry: &AutonomyRegistry,
    external_channels: &mut Vec<ChannelKey>,
) -> Result<Box<dyn crate::pipeline::node::PipelineNode>, PipelineAssemblyError> {
    // Dispatch on estimator family. Each family owns a different build
    // context shape (Gaussian needs aiding handlers; mock needs none;
    // particle will need particle-count / resampling). Adding a new
    // family means a new `registry/<family>.rs` and a new arm here.
    match est_cfg {
        EstimatorConfig::Ekf(ekf_cfg) => gaussian_estimator::assemble(
            instance_name,
            est_cfg,
            ekf_cfg,
            agent_handle,
            sensor_frame_handles,
            registry,
            external_channels,
        ),
        EstimatorConfig::Ukf(_) => Err(PipelineAssemblyError::FactoryFailure {
            node_kind: "Ukf".to_string(),
            reason: "UKF not yet implemented".to_string(),
        }),
        EstimatorConfig::MockOracle(_) => {
            // Mocks declare oracle inputs through their port descriptor and
            // the build-time check against BodyCapabilities decides whether
            // the body satisfies them. Nothing to push onto external_channels:
            // that list is for sensor / control signals routed around the
            // graph, not for body-published oracle channels.
            registry
                .build_mock_estimator(
                    "MockOracle",
                    est_cfg.clone(),
                    MockEstimatorBuildContext {
                        agent_handle,
                        instance_name: instance_name.to_string(),
                    },
                )
                .map_err(|reason| PipelineAssemblyError::FactoryFailure {
                    node_kind: "MockOracle".to_string(),
                    reason,
                })
        }
    }
}
