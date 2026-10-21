use crate::brain_bridge::components::{
    AgentIdComponent, AutonomyPipelineComponent, MissionGoalChannels, PipelineBuildFailed,
    SensorPublishChannel, TeleopControlled, TfServiceComponent,
};
use crate::core::components::{ActuatorCommandComponent, ControllerStateSource};
use crate::core::ground_truth::{truth_body_channels, DATUM_CONVENTION};
use crate::core::transforms::{TfTree, TrackedFrame};
use crate::prelude::*;
use crate::registry::plugin::RuntimeAutonomyRegistry;

use helios_core::control::actuation_model::ActuationModel;
use helios_core::control::actuators::ActuatorCommand;
use helios_core::control::commands::TwistIntent;
use helios_core::prelude::{AgentId, MonotonicTime};
use helios_core::spatial::transforms::tf::buffer::TfWindow;
use helios_core::spatial::transforms::tf::stamped::StampedTransform;
use helios_core::spatial::transforms::{Convention, EdgeEnds, ErasedTransform};
use helios_core::spatial::FrameId;
use helios_runtime::channels::control;
use helios_runtime::tf_service::TfService;
use helios_runtime::{
    build_pipeline, AutonomyPipeline, BodyCapabilities, BodyChannel, ChannelKey, HostDeclaration,
};

use std::any::TypeId;
use std::collections::{BTreeSet, HashMap};

/// Spawns the autonomy pipeline for agents with real estimation.
///
/// Collects the [`SensorPublishChannel`]s of each agent's descendants, and
/// the [`TrackedFrame`]s of the agent and its descendants, into the body
/// passed to `build_pipeline()`. Descendants, not children: a sensor entity
/// may hang under its installation's frame entity, as an IMU's two do. The
/// host declares its tick rate, the scenario's, and the world's datum.
#[allow(clippy::too_many_arguments)] // one Bevy system, each parameter a query or resource it reads
pub fn spawn_autonomy_pipeline(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest, &AgentIdComponent)>,
    children: Query<&Children>,
    channel_query: Query<&SensorPublishChannel>,
    frame_query: Query<&TrackedFrame>,
    registry: Res<RuntimeAutonomyRegistry>,
    config: Res<ScenarioConfig>,
    tf_tree: Res<TfTree>,
) {
    let _span = tracing::info_span!("sim.scene_build.autonomy").entered();
    let host = host_declaration(config.common.simulation.frequency_hz, tf_tree.datum());
    for (agent_entity, request, agent_id) in &agent_query {
        let agent_config = &request.0;
        let stack = agent_config.autonomy_stack();
        // The canonical scope stamped on the shell at `CreateRequests`; the
        // static seeds name every installation's frame under it, matching what
        // the sensor spawners stamped on their frame entities.
        let agent = agent_id.0.clone();

        // Every channel this agent's sensors publish, as their spawners
        // described them.
        let sensor_channels: Vec<BodyChannel> = children
            .iter_descendants(agent_entity)
            .filter_map(|descendant| channel_query.get(descendant).ok())
            .map(|channel| channel.description().clone())
            .collect();

        // Every frame of the body, as its spawners tracked it: `base_link` on
        // the agent, each installation and mount below it.
        let frames: HashMap<FrameId, Convention> = std::iter::once(agent_entity)
            .chain(children.iter_descendants(agent_entity))
            .filter_map(|entity| frame_query.get(entity).ok())
            .map(|frame| (frame.id.clone(), frame.convention))
            .collect();

        let body = build_host_body_capabilities(
            &agent,
            sensor_channels,
            frames,
            &agent_config.vehicle.actuation,
            host.tick_rate_hz,
        );

        // A pipeline is bound only with a tf service that serves every mount
        // it reads; either failing leaves the agent without both.
        let bound = build_pipeline(stack, &registry.0, agent.clone(), body, host.clone())
            .map_err(|errors| errors.iter().map(ToString::to_string).collect())
            .and_then(|pipeline| {
                let static_seeds = build_static_seeds(agent_config, &agent);
                estimated_tf_service(&pipeline, stack.tf.to_window(), static_seeds, &agent)
                    .map(|service| (pipeline, service))
            });

        match bound {
            Ok((pipeline, service)) => {
                // `AgentIdComponent` is already on the shell (stamped at
                // `CreateRequests`), so the test bridge's
                // `(&AgentIdComponent, &AutonomyPipelineComponent)` query still
                // never sees a pipeline without its identity.
                //
                // The distinct channels this agent's planners read their goal
                // from; a mission goal fans out to each. Empty when no node
                // declares a goal input.
                let goal_channels = mission_goal_channels(pipeline.outside_inputs());
                let teleop_controlled = reads_teleop_intent(pipeline.outside_inputs());

                let mut cmds = commands.entity(agent_entity);
                cmds.insert(AutonomyPipelineComponent(pipeline));
                cmds.insert(TfServiceComponent(service));

                if !goal_channels.is_empty() {
                    cmds.insert(MissionGoalChannels(goal_channels.into_iter().collect()));
                }
                if teleop_controlled {
                    cmds.insert(TeleopControlled);
                }

                info!(
                    "[spawn_autonomy_pipeline] Built pipeline for agent '{}'",
                    agent_config.name()
                );
            }
            Err(errors) => {
                for err in &errors {
                    error!(
                        "[spawn_autonomy_pipeline] Agent '{}': {}",
                        agent_config.name(),
                        err
                    );
                }
                // Mark the agent so the failure outlives the log line. The
                // agent gets no pipeline, so nothing downstream would
                // otherwise notice it is inert.
                commands
                    .entity(agent_entity)
                    .insert(PipelineBuildFailed { errors });
            }
        }
    }
}

/// The agent's estimated tf service, checked against `pipeline`.
///
/// Seeded with the static sensor mounts and draining the pipeline's `tf`
/// channel, so its tree is fed only by the graph's own producers, never the
/// sim's truth. Fails with every rejected seed, or with the mounts the
/// pipeline reads that are not below `base_link` or that a node reads in other
/// axes than the tree holds. A producer declares each edge's child but not its
/// parent, so no edge is declared yet; the one today, `odom → base_link`, lies
/// above `base_link`, on no mount's way to it.
fn estimated_tf_service(
    pipeline: &AutonomyPipeline,
    window: TfWindow,
    static_seeds: Vec<StampedTransform>,
    agent: &AgentId,
) -> Result<TfService, Vec<String>> {
    let service = TfService::new(window, static_seeds)
        .map_err(|rejected| rejected.iter().map(ToString::to_string).collect::<Vec<_>>())?;
    service
        .check_read_mounts(
            &pipeline.read_mounts(),
            &FrameId::base_link(agent.clone()),
            &[],
        )
        .map_err(|refused| vec![refused.to_string()])?;
    Ok(service)
}

/// The static transform edges seeding this agent's estimated tf buffer: one
/// `base_link → installation` edge per sensor installation, built from the
/// same [`frame`](crate::config::structs::SensorInstallationConfig::frame) the
/// sensor spawners stamp on their frame entities.
///
/// Single-sourcing the mount here (rather than reading it back off the sim's
/// entities) is what keeps the estimated tree and the sim from disagreeing on a
/// mount while leaving the estimated buffer structurally unable to reach the
/// sim's truth. Each mount is time-invariant, so it is stamped at `t = 0` and folded
/// once when the [`TfService`] is constructed. Each edge's child is the
/// installation, in its device's convention, and its parent `base_link`, in
/// the body convention its topology declares.
fn build_static_seeds(agent_config: &AgentConfig, agent: &AgentId) -> Vec<StampedTransform> {
    let base_link = FrameId::base_link(agent.clone());
    let body_convention = agent_config.vehicle.topology.convention();

    agent_config
        .sensors
        .iter()
        .map(|(key, installation)| installation.frame(agent, key))
        .map(|frame| StampedTransform {
            transform: ErasedTransform::new(
                frame.pose.to_isometry(),
                EdgeEnds {
                    child: frame.convention,
                    parent: body_convention,
                },
            ),
            child: frame.id,
            parent: base_link.clone(),
            stamp: MonotonicTime(0.0),
        })
        .collect()
}

pub fn spawn_actuator_command(
    mut commands: Commands,
    agent_query: Query<Entity, With<AutonomyPipelineComponent>>,
) {
    for entity in &agent_query {
        commands.entity(entity).insert((
            ActuatorCommandComponent(ActuatorCommand::new(vec![])),
            // Required by the vehicle HUD and toggled with T at runtime.
            // GroundTruth is the safer default so the controller sees real
            // physics state until the estimator has spun up.
            ControllerStateSource::GroundTruth,
        ));
    }
}

/// The sim host's [`HostDeclaration`]: it ticks at the scenario's
/// `tick_rate_hz`, and names one world frame, the datum truth is published
/// in, once the world layout has set it.
fn host_declaration(tick_rate_hz: f64, datum: Option<&FrameId>) -> HostDeclaration {
    HostDeclaration {
        tick_rate_hz,
        world_frames: datum
            .map(|datum| (datum.clone(), DATUM_CONVENTION))
            .into_iter()
            .collect(),
    }
}

/// Builds the sim host's [`BodyCapabilities`] for one agent: everything the
/// body offers the autonomy graph, distinct from anything the autonomy stack
/// itself declares.
///
/// - `publishes`: the truth rig's two channels for `agent`
///   ([`truth_body_channels`]), written every tick at `tick_rate_hz`, then
///   `sensor_channels`, each sensor's channel as its spawner described it.
///   Every installed sensor is listed, read by the stack or not. Health
///   channels are intentionally omitted: no sim publisher exists for them
///   yet, and advertising a channel the host never writes would let a
///   consumer build and then read `None` forever.
/// - `frames`: the body's frames as its spawners tracked them.
/// - `actuation`: the vehicle's actuators, from its entity file. The
///   assembler checks every actuator the pipeline drives against it, so an
///   allocator naming a missing actuator or writing the wrong setpoint kind
///   fails the build.
fn build_host_body_capabilities(
    agent: &AgentId,
    sensor_channels: Vec<BodyChannel>,
    frames: HashMap<FrameId, Convention>,
    actuation: &ActuationModel,
    tick_rate_hz: f64,
) -> BodyCapabilities {
    BodyCapabilities {
        publishes: truth_body_channels(agent, tick_rate_hz)
            .into_iter()
            .chain(sensor_channels)
            .collect(),
        frames,
        actuation: actuation.clone(),
    }
}

/// The channels a built pipeline expects a [`PlannerGoal`] on from outside,
/// read from its declared outside inputs.
///
/// A `BTreeSet` so planners sharing a channel (the common case: all on the
/// default `"mission"`) fold to one entry, and the order is stable across runs —
/// which the resolved-config dump and determinism hashing will want. Empty when
/// no node declares a goal input, so there is nowhere for a goal to go.
fn mission_goal_channels<'a>(
    outside_inputs: impl IntoIterator<Item = &'a ChannelKey>,
) -> BTreeSet<String> {
    outside_inputs
        .into_iter()
        .filter(|key| key.type_id() == TypeId::of::<PlannerGoal>())
        .map(|key| key.name().to_string())
        .collect()
}

/// Whether this agent's pipeline reads the operator's teleop intent. Only then
/// does the host publish intent to it: the pipeline's declared outside inputs
/// say what it reads from outside the robot, whatever its config calls the
/// node that reads it.
fn reads_teleop_intent<'a>(outside_inputs: impl IntoIterator<Item = &'a ChannelKey>) -> bool {
    let intent = control::intent::<TwistIntent>();
    outside_inputs.into_iter().any(|key| *key == intent)
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::control::actuation_model::{ActuatorSpec, SignConvention};
    use helios_core::control::actuators::{ActuatorId, SetpointKind};
    use helios_core::interchange::measurement::sensor::{GpsPosition, MagneticField};
    use helios_core::prelude::{AgentId, FrameId};
    use helios_runtime::channels::control::intent;
    use helios_runtime::port::ChannelKey;

    const TICK_RATE_HZ: f64 = 400.0;
    const GPS_RATE_HZ: f64 = 10.0;
    const MAG_RATE_HZ: f64 = 50.0;

    /// A goal input keyed the way a planner declares it: a `PlannerGoal` on the
    /// named channel.
    fn goal_key(goal_channel: &str) -> ChannelKey {
        ChannelKey::internal::<PlannerGoal>(goal_channel)
    }

    /// Every goal input's channel is collected — the mission goal fans out to
    /// all of them, not just the first one found.
    #[test]
    fn distinct_goal_channels_are_collected() {
        let outside_inputs = [goal_key("mission"), goal_key("waypoints")];

        assert_eq!(
            mission_goal_channels(&outside_inputs),
            BTreeSet::from(["mission".to_string(), "waypoints".to_string()]),
        );
    }

    /// Goal inputs sharing a channel (the common case: planners all on the
    /// default `"mission"`) fold to a single entry, so the host writes the goal
    /// once, not once per planner.
    #[test]
    fn goal_inputs_sharing_a_channel_are_deduped() {
        let outside_inputs = [goal_key("mission"), goal_key("mission")];

        let channels = mission_goal_channels(&outside_inputs);

        assert_eq!(channels.len(), 1);
        assert!(channels.contains("mission"));
    }

    /// With no goal input there is nowhere for a goal to go, so the set is
    /// empty — which is what lets `spawn_autonomy_pipeline` skip stamping the
    /// component.
    #[test]
    fn no_goal_inputs_yields_no_channels() {
        assert!(mission_goal_channels(&[]).is_empty());
    }

    /// Outside inputs of other types (teleop intent here) are not goal
    /// channels; only `PlannerGoal` keys are collected.
    #[test]
    fn other_outside_inputs_are_ignored() {
        let outside_inputs = [intent::<TwistIntent>(), goal_key("mission")];

        assert_eq!(
            mission_goal_channels(&outside_inputs),
            BTreeSet::from(["mission".to_string()]),
        );
    }

    /// A pipeline that declares the intent input is teleop controlled; the
    /// goal it also reads doesn't matter.
    #[test]
    fn declared_intent_marks_the_agent_teleop_controlled() {
        let outside_inputs = [goal_key("mission"), intent::<TwistIntent>()];

        assert!(reads_teleop_intent(&outside_inputs));
    }

    /// A pipeline that reads only goals gets no intent from the host.
    #[test]
    fn without_the_intent_input_the_agent_is_not_teleop_controlled() {
        assert!(!reads_teleop_intent(&[goal_key("mission")]));
        assert!(!reads_teleop_intent(&[]));
    }

    #[test]
    fn actuation_is_the_vehicles() {
        let actuation = ActuationModel::new(vec![ActuatorSpec::new(
            ActuatorId::new("drive"),
            SetpointKind::Torque,
            1.0,
            SetpointKind::Torque.value(0.0),
            SignConvention::Normal,
        )]);

        let caps = build_host_body_capabilities(
            &AgentId::new("car"),
            vec![],
            HashMap::new(),
            &actuation,
            TICK_RATE_HZ,
        );

        assert_eq!(caps.actuation, actuation);
    }

    /// The body carries the frames its spawners tracked, as given.
    #[test]
    fn frames_are_the_tracked_frames() {
        let agent = AgentId::new("car");
        let frames = HashMap::from([
            (FrameId::base_link(agent.clone()), Convention::Flu),
            (FrameId::agent(agent.clone(), "gps"), Convention::Enu),
        ]);

        let caps = build_host_body_capabilities(
            &agent,
            vec![],
            frames.clone(),
            &ActuationModel::default(),
            TICK_RATE_HZ,
        );

        assert_eq!(caps.frames, frames);
    }

    /// The truth rig writes every tick, so its channels carry the host's
    /// tick rate.
    #[test]
    fn the_truth_rig_writes_at_the_tick_rate() {
        let caps = build_host_body_capabilities(
            &AgentId::new("car"),
            vec![],
            HashMap::new(),
            &ActuationModel::default(),
            TICK_RATE_HZ,
        );

        assert!(caps
            .publishes
            .iter()
            .all(|channel| channel.rate_hz() == Some(TICK_RATE_HZ)));
    }

    /// The host ticks at the scenario's rate and names the datum once the
    /// world layout has set it, and no world frame before.
    #[test]
    fn the_host_declares_its_tick_and_the_datum() {
        let datum = FrameId::global("datum");

        let host = host_declaration(TICK_RATE_HZ, Some(&datum));
        assert_eq!(host.tick_rate_hz, TICK_RATE_HZ);
        assert_eq!(
            host.world_frames,
            HashMap::from([(datum, DATUM_CONVENTION)])
        );

        assert!(host_declaration(TICK_RATE_HZ, None).world_frames.is_empty());
    }

    /// The body publishes the truth rig's channels and every sensor's, each
    /// as described.
    #[test]
    fn publishes_the_truth_rig_and_every_sensor_channel() {
        let agent = AgentId::new("car");
        let gps = SensorPublishChannel::new::<GpsPosition>(
            "gps",
            "gps",
            FrameId::agent(agent.clone(), "gps"),
            GPS_RATE_HZ,
        );
        let mag = SensorPublishChannel::new::<MagneticField>(
            "mag",
            "mag",
            FrameId::agent(agent.clone(), "mag"),
            MAG_RATE_HZ,
        );
        let sensors = vec![gps.description().clone(), mag.description().clone()];

        let caps = build_host_body_capabilities(
            &agent,
            sensors.clone(),
            HashMap::new(),
            &ActuationModel::default(),
            TICK_RATE_HZ,
        );

        let expected: Vec<BodyChannel> = truth_body_channels(&agent, TICK_RATE_HZ)
            .into_iter()
            .chain(sensors)
            .collect();
        assert_eq!(caps.publishes, expected);
    }
}
