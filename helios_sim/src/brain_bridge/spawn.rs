use crate::brain_bridge::components::{
    AgentIdComponent, AutonomyPipelineComponent, MissionGoalChannels, OdomFrameOf,
    PipelineBuildFailed, SensorPublishChannel, TeleopControlled, TfServiceComponent,
};
use crate::core::components::{ActuatorCommandComponent, ControllerStateSource};
use crate::core::ground_truth::oracle_body_channels;
use crate::prelude::*;
use crate::registry::plugin::RuntimeAutonomyRegistry;

use helios_core::control::actuation_model::ActuationModel;
use helios_core::control::actuators::ActuatorCommand;
use helios_core::control::commands::TwistIntent;
use helios_core::prelude::{AgentId, MonotonicTime};
use helios_core::spatial::transforms::tf::buffer::TfWindow;
use helios_core::spatial::transforms::tf::stamped::StampedTransform;
use helios_core::spatial::transforms::{Convention, ErasedTransform};
use helios_core::spatial::FrameId;
use helios_runtime::channels::control;
use helios_runtime::tf_service::TfService;
use helios_runtime::{build_pipeline, AutonomyPipeline, BodyCapabilities, BodyChannel, ChannelKey};

use std::any::TypeId;
use std::collections::BTreeSet;

/// Spawns the autonomy pipeline for agents with real estimation.
///
/// Collects each agent's sensor children's [`SensorPublishChannel`]s into the
/// body passed to `build_pipeline()`.
pub fn spawn_autonomy_pipeline(
    mut commands: Commands,
    agent_query: Query<(
        Entity,
        &SpawnAgentConfigRequest,
        &AgentIdComponent,
        &Children,
    )>,
    channel_query: Query<&SensorPublishChannel>,
    registry: Res<RuntimeAutonomyRegistry>,
) {
    let _span = tracing::info_span!("sim.scene_build.autonomy").entered();
    for (agent_entity, request, agent_id, children) in &agent_query {
        let agent_config = &request.0;
        let stack = agent_config.autonomy_stack();
        // The canonical scope stamped on the shell at `CreateRequests`; the
        // assembler builds every sensor's `FrameId::agent(agent, channel)` from
        // it, matching what the sensor spawners stamped on the tf tree.
        let agent = agent_id.0.clone();

        // Every channel this agent's sensors publish, as their spawners
        // described them.
        let sensor_channels: Vec<BodyChannel> = children
            .iter()
            .filter_map(|child| channel_query.get(child).ok())
            .map(|channel| channel.description().clone())
            .collect();

        let body = build_host_body_capabilities(sensor_channels, &agent_config.vehicle.actuation);

        // A pipeline is bound only with a tf service that serves every mount
        // it reads; either failing leaves the agent without both.
        let bound = build_pipeline(stack, &registry.0, agent.clone(), body)
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
/// Seeded with the static sensor mounts and draining exactly the tf-edge
/// channels the pipeline declares, so its tree is fed only by the graph's own
/// producers, never the sim's truth tree. Fails with every rejected seed, or
/// with the mounts the pipeline reads that are not below `base_link`. The
/// pipeline's tf edges do not yet say which frames they join, so none is
/// declared; the one today, `odom → base_link`, lies above `base_link`, on no
/// mount's way to it.
fn estimated_tf_service(
    pipeline: &AutonomyPipeline,
    window: TfWindow,
    static_seeds: Vec<StampedTransform>,
    agent: &AgentId,
) -> Result<TfService, Vec<String>> {
    let service = TfService::new(window, pipeline.tf_edge_channels(), static_seeds)
        .map_err(|rejected| rejected.iter().map(ToString::to_string).collect::<Vec<_>>())?;
    service
        .check_mounts_reach(
            &pipeline.read_mounts(),
            &FrameId::base_link(agent.clone()),
            &[],
        )
        .map_err(|unreached| vec![unreached.to_string()])?;
    Ok(service)
}

/// The static transform edges seeding this agent's estimated tf buffer: one
/// `base_link → sensor` mount per sensor frame, built from the same sensor
/// configs the sensor spawners read.
///
/// Single-sourcing the mount here (rather than reading it back off the truth
/// tree) is what keeps the estimated and truth trees from disagreeing on a mount
/// while leaving the estimated buffer structurally unable to reach the truth
/// tree. Each mount is time-invariant, so it is stamped at `t = 0` and folded
/// once when the [`TfService`] is constructed. The pose is `sensor` expressed in
/// `base_link` (FLU); `from_parts` takes the sensor's own convention first, the
/// `base_link` convention (FLU) second.
fn build_static_seeds(agent_config: &AgentConfig, agent: &AgentId) -> Vec<StampedTransform> {
    let base_link = FrameId::base_link(agent.clone());

    agent_config
        .sensors
        .values()
        .flat_map(|sensor| sensor.frame_mounts())
        .map(|(channel, pose, convention)| StampedTransform {
            child: FrameId::agent(agent.clone(), channel),
            parent: base_link.clone(),
            stamp: MonotonicTime(0.0),
            transform: ErasedTransform::from_parts(pose.to_isometry(), convention, Convention::Flu),
        })
        .collect()
}

/// Spawns odom frame entities for agents that have an `AutonomyPipelineComponent`.
pub fn spawn_odom_frames(
    mut commands: Commands,
    agent_query: Query<(Entity, &AgentIdComponent), With<AutonomyPipelineComponent>>,
) {
    for (agent_entity, agent_id) in &agent_query {
        let agent = &agent_id.0;
        commands.spawn((
            Name::new(format!("{}/odom", agent)),
            // ENU because the estimator's world frame is ENU. This is a property
            // of the estimation stack, not the vehicle — every agent's odom shares
            // it regardless of body convention. A future NED-world estimator would
            // make this configurable at the estimator layer, not per vehicle.
            TrackedFrame::new(FrameId::odom(agent.clone()), Convention::Enu),
            Transform::IDENTITY,
            GlobalTransform::IDENTITY,
            OdomFrameOf(agent_entity),
        ));
        info!("[OdomFrame] Spawned odom frame for '{}'", agent);
    }
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

/// Builds the sim host's [`BodyCapabilities`] for one agent: everything the
/// body offers the autonomy graph, distinct from anything the autonomy stack
/// itself declares.
///
/// - `publishes`: the truth rig's two oracle channels
///   ([`oracle_body_channels`]), then `sensor_channels`, each sensor's
///   channel as its spawner described it. Every installed sensor is listed,
///   read by the stack or not. Health channels are intentionally omitted: no
///   sim publisher exists for them yet, and advertising a channel the host
///   never writes would let a consumer build and then read `None` forever.
/// - `actuation`: the vehicle's actuators, from its entity file. The
///   assembler checks every actuator the pipeline drives against it, so an
///   allocator naming a missing actuator or writing the wrong setpoint kind
///   fails the build.
fn build_host_body_capabilities(
    sensor_channels: Vec<BodyChannel>,
    actuation: &ActuationModel,
) -> BodyCapabilities {
    BodyCapabilities {
        publishes: oracle_body_channels()
            .into_iter()
            .chain(sensor_channels)
            .collect(),
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
        .map(|key| key.instance().to_string())
        .collect()
}

/// Whether this agent's pipeline reads the operator's teleop intent. Only then
/// does the host publish intent to it: the pipeline's declared outside inputs
/// say what it reads from outside the robot, whatever its config calls the
/// node that reads it.
fn reads_teleop_intent<'a>(outside_inputs: impl IntoIterator<Item = &'a ChannelKey>) -> bool {
    let intent: ChannelKey = control::intent::<TwistIntent>().into();
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
    use helios_runtime::port::InternalChannel;

    /// A goal input keyed the way a planner declares it: a `PlannerGoal` on the
    /// named channel.
    fn goal_key(goal_channel: &str) -> ChannelKey {
        InternalChannel::named::<PlannerGoal>(goal_channel).into()
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
        let outside_inputs = [intent::<TwistIntent>().into(), goal_key("mission")];

        assert_eq!(
            mission_goal_channels(&outside_inputs),
            BTreeSet::from(["mission".to_string()]),
        );
    }

    /// A pipeline that declares the intent input is teleop controlled; the
    /// goal it also reads doesn't matter.
    #[test]
    fn declared_intent_marks_the_agent_teleop_controlled() {
        let outside_inputs = [goal_key("mission"), intent::<TwistIntent>().into()];

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

        let caps = build_host_body_capabilities(vec![], &actuation);

        assert_eq!(caps.actuation, actuation);
    }

    /// The body publishes the truth rig's channels and every sensor's, each
    /// as described.
    #[test]
    fn publishes_the_oracle_and_every_sensor_channel() {
        let agent = AgentId::new("car");
        let gps = SensorPublishChannel::new::<GpsPosition>(
            "gps",
            "gps_pro",
            FrameId::agent(agent.clone(), "gps"),
        );
        let mag = SensorPublishChannel::new::<MagneticField>(
            "mag",
            "mag_basic",
            FrameId::agent(agent, "mag"),
        );
        let sensors = vec![gps.description().clone(), mag.description().clone()];

        let caps = build_host_body_capabilities(sensors.clone(), &ActuationModel::default());

        let expected: Vec<BodyChannel> =
            oracle_body_channels().into_iter().chain(sensors).collect();
        assert_eq!(caps.publishes, expected);
    }
}
