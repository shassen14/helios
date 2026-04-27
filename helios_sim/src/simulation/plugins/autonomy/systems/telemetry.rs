// helios_sim/src/simulation/plugins/autonomy/systems/telemetry.rs
//
// Telemetry systems (cold path — Validation phase).

use bevy::prelude::*;
use helios_runtime::stage::PipelineLevel;

use crate::simulation::core::components::{AgentTopicNames, SensorMailbox};
use crate::simulation::core::topics::TopicBus;
use crate::simulation::plugins::autonomy::components::{
    EstimatorComponent, MapperComponent, PathFollowingOutputComponent,
};

/// Publishes estimated state, maps, and active planning waypoint to TopicBus.
/// Runs in `SimulationSet::Validation` (cold telemetry path).
/// Topic name strings are pre-computed in `AgentTopicNames` — no `format!()` allocations here.
pub fn publish_autonomy_telemetry(
    query: Query<(
        &EstimatorComponent,
        &MapperComponent,
        &PathFollowingOutputComponent,
        &AgentTopicNames,
    )>,
    mut topic_bus: ResMut<TopicBus>,
) {
    for (estimator, mapper, pf_output, topics) in &query {
        // Active path following reference point.
        if let Some(ref wp) = pf_output.0 {
            topic_bus.publish(&topics.active_waypoint, wp.state.clone());
        }
        if let Some(state) = estimator.0.get_state() {
            topic_bus.publish(&topics.odometry_estimated, state.clone());
        }

        // Global map: SLAM takes priority over global mappers.
        let global_map = estimator
            .0
            .get_slam_map()
            .or_else(|| mapper.0.get_map(&PipelineLevel::Global));
        if let Some(map) = global_map {
            if !matches!(map, helios_core::mapping::MapData::None) {
                topic_bus.publish(&topics.map_global, map.clone());
            }
        }

        if let Some(map) = mapper.0.get_map(&PipelineLevel::Local) {
            if !matches!(map, helios_core::mapping::MapData::None) {
                topic_bus.publish(&topics.map_local, map.clone());
            }
        }
    }
}

/// Publishes each agent's sensor measurements to TopicBus.
/// Runs in `SimulationSet::Validation` (cold telemetry path).
/// Sensor systems no longer touch `TopicBus` directly — all sensor telemetry flows here.
pub fn publish_sensor_telemetry(query: Query<&SensorMailbox>, mut topic_bus: ResMut<TopicBus>) {
    for mailbox in &query {
        for entry in &mailbox.entries {
            if !entry.topic_name.is_empty() {
                topic_bus.publish(&entry.topic_name, entry.message.clone());
            }
        }
    }
}
