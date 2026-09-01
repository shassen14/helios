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
    command_sum_node_name, selector_policy, REFERENCE_ARBITER_NODE, TELEOP_MAPPER_NODE,
};
use crate::body::{BodyCapabilities, Provenance, PublishedChannel};
use crate::channels::control;
use crate::config::TeleopMapperConfig;
use crate::config::{AllocatorConfig, AutonomyStack, CommandSpace, FoldRole, ReferenceSource};
use crate::config::{EstimatorConfig, MapLayerConfig};
use crate::nodes::combinators::{Merge, Selector, Sum};
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

use helios_core::control::actuators::ActuatorCommand;
use helios_core::control::commands::{BodyTwist, DriveForce, SteerAngle, TwistIntent};
use helios_core::control::BodyTwistRef;
use helios_core::data::primitives::FrameHandle;
use helios_core::frames::FrameAwareState;
use helios_core::mapping::MapData;
use helios_core::planning::types::Path;

use std::collections::{BTreeSet, HashMap, HashSet};
use std::ops::Add;

/// Node name for the synthesized actuator merge — the terminal that unions each
/// allocator's partial [`ActuatorCommand`] into the one `actuators` output. Raw
/// identity for observability, so a referenced const like the command-seam node
/// names; `build_pipeline` here is its sole synthesizer. It names the actuator
/// terminal rather than the command seam, so it lives here, not in
/// [`self::command`] with the fold and arbiter names.
const ACTUATOR_MERGE_NODE: &str = "actuator_merge";

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

    // --- Path follower + guidance reference seam ---
    // The guidance reference is the single-signal seam where teleop and autonomy
    // are arbitrated — below all planning, above the controllers that track it.
    // The follower (autonomy) and the teleop mapper each write their own contender
    // role; a `Selector` resolves the fresher onto the resolved `reference` channel
    // the controllers read. Arbitration is synthesized only when *both* contend: a
    // lone source (a follower with no teleop, or teleop with no follower) writes
    // the resolved `reference` directly and no arbiter is needed — the analog of a
    // lone `Direct` command source. The seam type is `BodyTwistRef`, the only
    // reference type today; when a second appears, derive it from the follower's
    // declared reference rather than hard-coding it here.
    let teleop_contends = stack
        .reference_arbitration
        .sources
        .contains(&ReferenceSource::Teleop);
    let arbitrate_reference = teleop_contends && stack.path_following.is_some();

    if let Some(pf_cfg) = &stack.path_following {
        let follower_output = if arbitrate_reference {
            control::reference_autonomy::<BodyTwistRef>()
        } else {
            control::reference::<BodyTwistRef>()
        };

        match path_follower::resolve_path_channel(stack) {
            Ok(path_channel) => {
                match registry.build_path_follower(
                    pf_cfg.get_kind_str(),
                    PathFollowerBuildContext {
                        agent_handle,
                        config: pf_cfg.clone(),
                        path_channel,
                        output_channel: follower_output,
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

    // Teleop guidance ingress + reference arbiter. Teleop enters at the top of the
    // tracking layer, substituting for the follower's instantaneous output. Its
    // mapper scales the host's `intent` into a `BodyTwistRef` — onto the teleop
    // contender role when a follower also contends, else onto the resolved
    // `reference` directly. When both contend the arbiter picks the fresher of the
    // two onto `reference`; neutral intent publishes nothing, so the follower
    // reclaims the seam by freshness with no explicit release.
    if teleop_contends {
        let teleop_output = if arbitrate_reference {
            control::reference_teleop::<BodyTwistRef>()
        } else {
            control::reference::<BodyTwistRef>()
        };

        match &stack.teleop {
            Some(TeleopMapperConfig::Twist {
                surge,
                sway,
                heave,
                roll,
                pitch,
                yaw,
            }) => {
                let mapper = TwistTeleopNode::new(
                    TELEOP_MAPPER_NODE,
                    control::intent::<TwistIntent>(),
                    teleop_output,
                    TwistScale {
                        surge: *surge,
                        sway: *sway,
                        heave: *heave,
                        roll: *roll,
                        pitch: *pitch,
                        yaw: *yaw,
                    },
                );
                builder = builder.add_node(Box::new(mapper));
                external_channels.push(control::intent::<TwistIntent>().into());

                if arbitrate_reference {
                    let arbiter = Selector::<BodyTwistRef>::new(
                        REFERENCE_ARBITER_NODE,
                        vec![control::reference_teleop::<BodyTwistRef>()],
                        control::reference_autonomy::<BodyTwistRef>(),
                        control::reference::<BodyTwistRef>(),
                        selector_policy(&stack.reference_arbitration),
                    );
                    builder = builder.add_node(Box::new(arbiter));
                }
            }
            None => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: "TeleopMapper".to_string(),
                reason: "teleop is a declared command source but no [teleop] mapper \
                         config was provided"
                    .to_string(),
            }),
        }
    }

    // --- Controllers + command terminal ---
    // Each allocator consumes one command space, so the terminal is wired per
    // space the stack's allocators consume. A decoupled stack opens several seams
    // — a drive space and a steer space — and each gets its own `Sum` fold of the
    // controllers speaking it. The body-twist path is the default: it serves the
    // body-twist allocator and every no-sum-space stack (pure-perception,
    // controllers-without-allocator), which is why the branch turns on the
    // presence of a sum space, not on the allocator set being non-empty. Teleop is
    // arbitrated at the reference seam above, so it no longer factors in here.
    let allocator_spaces: BTreeSet<CommandSpace> = stack
        .allocators
        .values()
        .map(AllocatorConfig::command_space)
        .collect();

    let has_sum_space = allocator_spaces.contains(&CommandSpace::DriveForce)
        || allocator_spaces.contains(&CommandSpace::SteerAngle);

    builder = if has_sum_space {
        // One `Sum` per present sum space; each folds only its own controllers.
        let mut b = builder;
        if allocator_spaces.contains(&CommandSpace::DriveForce) {
            b = wire_sum_terminal::<DriveForce>(
                stack,
                registry,
                agent_handle,
                CommandSpace::DriveForce,
                b,
                &mut errors,
            );
        }

        if allocator_spaces.contains(&CommandSpace::SteerAngle) {
            b = wire_sum_terminal::<SteerAngle>(
                stack,
                registry,
                agent_handle,
                CommandSpace::SteerAngle,
                b,
                &mut errors,
            );
        }
        b
    } else {
        wire_body_twist_terminal(stack, registry, agent_handle, builder, &mut errors)
    };

    // --- Allocators + actuator merge ---
    // Each allocator *consumes* its space's `command` (the fold / arbiter above
    // produce it) and *produces* a partial `actuators` command over the actuators
    // it drives. Every channel here is graph-internal, so nothing is seeded onto
    // `external_channels`.
    //
    // Rather than write the `actuators` terminal directly, each allocator writes a
    // distinct partial keyed by its instance name; a single `Merge` unions the
    // partials into the terminal. This is the single shape for any allocator count
    // — a lone allocator flows through a one-input `Merge` that forwards it — so a
    // decoupled stack (a drive leg and a steer leg) reassembles the same way a
    // one-allocator stack does. Validation has already proven the partials own
    // disjoint actuators; `Merge` re-checks defensively and degrades on overlap.
    //
    // The command channel's type is the allocator's own command space, erased to an
    // `InternalChannel` so the loop is one shape regardless of `T`.
    let mut partials: Vec<InternalChannel> = vec![];
    for (allocator_name, alloc_cfg) in &stack.allocators {
        let command_channel = match alloc_cfg.command_space() {
            CommandSpace::DriveForce => control::command::<DriveForce>(),
            CommandSpace::SteerAngle => control::command::<SteerAngle>(),
            CommandSpace::BodyTwist => control::command::<BodyTwist>(),
        };

        let partial = InternalChannel::named::<ActuatorCommand>(allocator_name.as_str());

        match registry.build_allocator(
            alloc_cfg.get_kind_str(),
            AllocatorBuildContext {
                agent_handle,
                instance_name: allocator_name.clone(),
                config: alloc_cfg.clone(),
                input_channel: command_channel,
                output_channel: partial.clone(),
            },
        ) {
            Ok(node) => {
                builder = builder.add_node(node);
                partials.push(partial);
            }
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: alloc_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    if !partials.is_empty() {
        builder = builder.add_node(Box::new(Merge::new(
            ACTUATOR_MERGE_NODE,
            partials,
            control::actuators(),
        )));
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

/// Wires the body-twist command terminal for a coupled morphology: one command
/// space fed directly by the autonomy controllers. This serves a body-twist
/// allocator and every no-sum-space stack (pure-perception, controllers without
/// an allocator).
///
/// Teleop no longer enters here — it is arbitrated one layer up at the guidance
/// reference seam — so this path is autonomy-only. Each controller writes the
/// `command` terminal directly; a pure-perception stack has no controller and so
/// wires nothing. A coupled stack that folds multiple contributions would grow a
/// `Sum` here, the way the decoupled spaces do; the shipped coupled path is a
/// single controller, so none is synthesized yet.
fn wire_body_twist_terminal(
    stack: &AutonomyStack,
    registry: &AutonomyRegistry,
    agent_handle: FrameHandle,
    mut builder: PipelineBuilder,
    errors: &mut Vec<PipelineAssemblyError>,
) -> PipelineBuilder {
    for (controller_name, ctrl_cfg) in &stack.controllers {
        match registry.build_controller(
            ctrl_cfg.get_kind_str(),
            ControllerBuildContext {
                agent_handle,
                instance_name: controller_name.clone(),
                config: ctrl_cfg.clone(),
                output_channel: control::command::<BodyTwist>(),
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

    builder
}

/// Wires the autonomy-only command terminal for a single-command-space stack,
/// folding every controller's contribution into one `command` of type `T`.
///
/// Each controller writes its own instance-named contribution channel, and a
/// [`Sum`] folds them into the `command` terminal — a feedback and a feedforward
/// leg composing without clobbering. Feedback legs are the fold's `required`
/// inputs (read fresh); feedforward legs are `optional` (folded last-known-good).
/// No teleop, no arbiter: those await the arbiter's own command-space generalization.
///
/// `T` is the allocator's command space — [`DriveForce`] for the longitudinal
/// drive terminal, [`SteerAngle`] for the steer terminal. The body-twist stack
/// keeps its own richer arbiter path in [`wire_body_twist_terminal`], so it is
/// not one of the `T`s folded here.
fn wire_sum_terminal<T>(
    stack: &AutonomyStack,
    registry: &AutonomyRegistry,
    agent_handle: FrameHandle,
    space: CommandSpace,
    mut builder: PipelineBuilder,
    errors: &mut Vec<PipelineAssemblyError>,
) -> PipelineBuilder
where
    T: Send + Sync + Clone + Add<Output = T> + 'static,
{
    let mut required: Vec<InternalChannel> = vec![];
    let mut optional: Vec<InternalChannel> = vec![];

    for (controller_name, ctrl_cfg) in &stack.controllers {
        if ctrl_cfg.command_space() != space {
            continue;
        }
        // Distinct channel per producer (the planner precedent): a shared output
        // would clobber, since the bus keeps only the last write.
        let contribution = InternalChannel::named::<T>(controller_name.as_str());

        match registry.build_controller(
            ctrl_cfg.get_kind_str(),
            ControllerBuildContext {
                agent_handle,
                instance_name: controller_name.clone(),
                config: ctrl_cfg.clone(),
                output_channel: contribution.clone(),
            },
        ) {
            Ok(node) => {
                builder = builder.add_node(node);
                match ctrl_cfg.fold_role() {
                    FoldRole::Feedback => required.push(contribution),
                    FoldRole::Feedforward => optional.push(contribution),
                }
            }
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: ctrl_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    // No controllers ⇒ no fold: leave `command` unwritten so the allocator's
    // unsatisfied input surfaces as a loud build error rather than a silent
    // no-command. A degenerate stack is validation's job to reject.
    if !required.is_empty() || !optional.is_empty() {
        let sum = Sum::<T>::new(
            command_sum_node_name(space),
            required,
            optional,
            control::command::<T>(),
        );
        builder = builder.add_node(Box::new(sum));
    }

    builder
}

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
