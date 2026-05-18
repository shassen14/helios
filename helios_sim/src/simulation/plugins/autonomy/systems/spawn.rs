// helios_sim/src/simulation/plugins/autonomy/systems/spawn.rs
//
// Spawning systems for agent autonomy pipelines.

use bevy::prelude::*;
use std::collections::HashMap;

use helios_core::data::primitives::FrameHandle;
use helios_runtime::build_pipeline;

use crate::prelude::*;
use crate::simulation::plugins::autonomy::components::{
    AutonomyPipelineComponent, OdomFrameOf, SensorPublishChannel,
};
use crate::simulation::registry::plugin::RuntimeAutonomyRegistry;

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

        match build_pipeline(stack, &registry.0, agent_handle, &sensor_frame_handles) {
            Ok(pipeline) => {
                commands
                    .entity(agent_entity)
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
