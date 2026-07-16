use crate::brain_bridge::components::{
    AgentIdComponent, AutonomyPipelineComponent, OdomFrameOf, SensorPublishChannel,
};
use crate::core::components::{ControlOutputComponent, ControllerStateSource};
use crate::prelude::*;
use crate::registry::plugin::RuntimeAutonomyRegistry;

use helios_core::data::primitives::FrameHandle;
use helios_runtime::channels::{oracle_pose_channel, oracle_twist_channel};
use helios_runtime::{build_pipeline, BodyCapabilities, Provenance, PublishedChannel};

use nalgebra::Vector3;
use std::collections::HashMap;

/// Spawns the autonomy pipeline for agents with real estimation.
///
/// Queries each agent's sensor children for `SensorPublishChannel` to build
/// the channel→FrameHandle map passed to `build_pipeline()`.
pub fn spawn_autonomy_pipeline(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest, &Children)>,
    channel_query: Query<&SensorPublishChannel>,
    registry: Res<RuntimeAutonomyRegistry>,
) {
    let _span = tracing::info_span!("sim.scene_build.autonomy").entered();
    for (agent_entity, request, children) in &agent_query {
        let agent_config = &request.0;
        let stack = agent_config.autonomy_stack();
        let agent_handle = FrameHandle::from_entity(agent_entity);

        let sensor_frame_handles: HashMap<String, FrameHandle> = children
            .iter()
            .filter_map(|child| {
                channel_query
                    .get(child)
                    .ok()
                    .map(|ch| (ch.0.clone(), FrameHandle::from_entity(child)))
            })
            .collect();

        let host_capabilities = build_host_body_capabilities(agent_config.name());

        match build_pipeline(
            stack,
            &registry.0,
            agent_handle,
            &sensor_frame_handles,
            host_capabilities,
        ) {
            Ok(pipeline) => {
                // Insert the id in the same chain as the pipeline so the test
                // bridge's `(&AgentId, &AutonomyPipelineComponent)` query can
                // never see one without the other.
                commands
                    .entity(agent_entity)
                    .insert(AgentIdComponent(agent_config.name().to_string()))
                    .insert(AutonomyPipelineComponent(pipeline));
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
            }
        }
    }
}

/// Spawns odom frame entities for agents that have an `AutonomyPipelineComponent`.
pub fn spawn_odom_frames(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest), With<AutonomyPipelineComponent>>,
) {
    for (agent_entity, request) in &agent_query {
        let agent_name = request.0.name();
        commands.spawn((
            Name::new(format!("{}/odom", agent_name)),
            TrackedFrame,
            Transform::IDENTITY,
            GlobalTransform::IDENTITY,
            OdomFrameOf(agent_entity),
        ));
        info!("[OdomFrame] Spawned odom frame for '{}'", agent_name);
    }
}

pub fn spawn_control_output(
    mut commands: Commands,
    agent_query: Query<Entity, With<AutonomyPipelineComponent>>,
) {
    for entity in &agent_query {
        commands.entity(entity).insert((
            ControlOutputComponent(ControlOutput::BodyVelocity {
                linear: Vector3::zeros(),
                angular: Vector3::zeros(),
            }),
            // Required by the vehicle HUD and toggled with T at runtime.
            // GroundTruth is the safer default so the controller sees real
            // physics state until the estimator has spun up.
            ControllerStateSource::GroundTruth,
        ));
    }
}

/// Builds the sim host's [`BodyCapabilities`] for one agent.
///
/// This is the host's view of what the body advertises to the autonomy
/// graph — distinct from anything the autonomy stack itself declares.
/// Two responsibilities split between host and assembler:
/// - **Host (this fn)** knows the agent's name, whether the body actuates,
///   and which host-owned channels exist (`oracle/*`, later `health/*`).
/// - **Assembler** ([`build_pipeline`]) knows the sensor channels derived
///   from the autonomy config and merges them into `publishes` before
///   handing the merged `BodyCapabilities` to [`PipelineBuilder`].
///
/// # Field rationale
///
/// - `name`: cloned from `agent_name`; used by error messages
///   (`PipelineBuildError::UnsatisfiedBodyCapabilities`) and the DAG dump.
/// - `consumes_control`: hardcoded `true` — every sim agent has an
///   actuation adapter (`AckermannAdapterComponent` etc.). Becomes a
///   parameter (or moves to `AgentConfig`) once passive observer agents,
///   log-playback agents, or hw-passive Zenoh-bridge nodes exist.
/// - `publishes`: only the two oracle channels — `oracle/pose` (world ENU)
///   and `oracle/twist` (body FLU). Both tagged `Provenance::Exact`
///   because the sim is ground truth. Health channels are intentionally
///   omitted: no sim publisher exists for them yet, and advertising a
///   channel the host never writes would let a consumer build and then
///   read `None` forever.
///
/// Construct one of these in [`spawn_autonomy_pipeline`] per agent and
/// pass it to [`build_pipeline`].
fn build_host_body_capabilities(agent_name: &str) -> BodyCapabilities {
    let published_pose = PublishedChannel {
        key: oracle_pose_channel(),
        provenance: Provenance::Exact,
    };

    let published_twist = PublishedChannel {
        key: oracle_twist_channel(),
        provenance: Provenance::Exact,
    };

    BodyCapabilities {
        name: agent_name.to_string(),
        publishes: vec![published_pose, published_twist],
        consumes_control: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn name_is_copied_from_argument() {
        let caps = build_host_body_capabilities("rover_1");
        assert_eq!(caps.name, "rover_1");
    }

    #[test]
    fn sim_agents_consume_control() {
        let caps = build_host_body_capabilities("any");
        assert!(caps.consumes_control);
    }

    #[test]
    fn publishes_exactly_pose_and_twist() {
        let caps = build_host_body_capabilities("any");
        assert_eq!(caps.publishes.len(), 2);

        let keys: Vec<_> = caps.publishes.iter().map(|p| &p.key).collect();
        assert!(keys.contains(&&oracle_pose_channel()));
        assert!(keys.contains(&&oracle_twist_channel()));
    }

    #[test]
    fn all_published_channels_are_provenance_exact() {
        let caps = build_host_body_capabilities("any");
        for pc in &caps.publishes {
            assert_eq!(pc.provenance, Provenance::Exact);
        }
    }
}
