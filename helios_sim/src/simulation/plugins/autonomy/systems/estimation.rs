// helios_sim/src/simulation/plugins/autonomy/systems/estimation.rs
//
// Sensor routing, estimation, and odom frame update systems.

use bevy::prelude::*;

use crate::simulation::core::components::{MailboxEntry, SensorMailbox, SensorTopicName};
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::sim_runtime::SimRuntime;
use crate::simulation::core::transforms::{EnuBodyPose, TfTree};
use crate::simulation::plugins::autonomy::components::{EstimatorComponent, OdomFrameOf};

/// Routes `BevyMeasurementMessage` events to per-agent `SensorMailbox` components.
/// Clears each mailbox, fills it with this frame's messages, and sorts by timestamp.
/// Must run before `run_estimation` and `run_mapping`.
pub fn route_sensor_messages(
    mut events: EventReader<BevyMeasurementMessage>,
    mut mailbox_query: Query<&mut SensorMailbox>,
    topic_name_query: Query<&SensorTopicName>,
) {
    // Clear all mailboxes at the start of each frame.
    for mut mailbox in &mut mailbox_query {
        mailbox.entries.clear();
    }

    for event in events.read() {
        let sensor_entity = event.0.sensor_handle.to_entity();
        let topic_name = topic_name_query
            .get(sensor_entity)
            .map(|t| t.0.clone())
            .unwrap_or_default();

        let agent_entity = event.0.agent_handle.to_entity();
        if let Ok(mut mailbox) = mailbox_query.get_mut(agent_entity) {
            mailbox.entries.push(MailboxEntry {
                topic_name,
                message: event.0.clone(),
            });
        }
    }

    // Sort each mailbox by timestamp (ascending) for EKF causality.
    for mut mailbox in &mut mailbox_query {
        mailbox.entries.sort_by(|a, b| {
            a.message
                .timestamp
                .partial_cmp(&b.message.timestamp)
                .unwrap()
        });
    }
}

/// Processes all measurements in each agent's `SensorMailbox` through the estimator.
/// Runs in parallel with `run_mapping` — accesses `EstimatorComponent` only.
pub fn run_estimation(
    mut agent_query: Query<(&mut EstimatorComponent, &SensorMailbox)>,
    tf_tree: Res<TfTree>,
    time: Res<Time>,
) {
    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    for (mut estimator, mailbox) in &mut agent_query {
        for entry in &mailbox.entries {
            estimator.0.process_measurement(&entry.message, &runtime);
        }
    }
}

pub fn update_odom_frames(
    agent_query: Query<&EstimatorComponent>,
    mut odom_query: Query<(&OdomFrameOf, &mut Transform)>,
) {
    for (odom_of, mut transform) in &mut odom_query {
        let Ok(estimator) = agent_query.get(odom_of.0) else {
            continue;
        };

        if let Some(iso) = estimator.0.get_state().and_then(|s| s.get_pose_isometry()) {
            *transform = Transform::from(EnuBodyPose(iso));
        }
    }
}
