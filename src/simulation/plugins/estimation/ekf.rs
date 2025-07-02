// use crate::simulation::core::dynamics::DynamicsModel;
// use crate::simulation::core::topics::{EstimatedPose, ImuData};
// use bevy::prelude::*;
// use std::collections::HashMap;

// // You will need to define DebugCounter in your example and add it to your prelude,
// // or define it in a core file. For this example, we assume it's findable.
// use crate::prelude::DebugCounter; // Assuming it's in the prelude

// --- Public SystemSet for external coordination ---
// #[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
// pub enum EkfSystems {
//     Tick,
//     Main,
// }

// // --- Plugin Definition ---
// pub struct EkfPlugin;

// impl Plugin for EkfPlugin {
//     fn build(&self, app: &mut App) {
//         app
//             // 1. Add the ticking system to its own set.
//             .add_systems(FixedUpdate, tick_ekf_timers.in_set(EkfSystems::Tick))
//             // 2. Add the main EKF logic systems to another set.
//             .add_systems(
//                 FixedUpdate,
//                 (
//                     ekf_predict_system,
//                     ekf_update_from_imu.after(ekf_predict_system),
//                     ekf_write_output.after(ekf_update_from_imu),
//                 )
//                     .in_set(EkfSystems::Main)
//                     // The entire block is gated by this single run condition.
//                     .run_if(should_run_ekf_system),
//             )
//             // 3. Explicitly state that the Main set must run after the Tick set.
//             .configure_sets(FixedUpdate, EkfSystems::Main.after(EkfSystems::Tick));
//     }
// }

// // --- Components ---
// #[derive(Component)]
// pub struct EKF {
//     pub timer: Timer,
// }

// #[derive(Component, Default)]
// pub struct EkfState {
//     // In a real EKF, this would hold your state vector and covariance matrix.
//     // DVector<f64>, DMatrix<f64>, etc.
// }

// // --- Systems ---

// /// Ticks all EKF timers. This system has write access and runs every `FixedUpdate`.
// fn tick_ekf_timers(mut query: Query<&mut EKF>, time: Res<Time>) {
//     for mut ekf in query.iter_mut() {
//         ekf.timer.tick(time.delta());
//     }
// }

// /// The READ-ONLY run condition. Returns true if *any* EKF timer has finished.
// fn should_run_ekf_system(query: Query<&EKF>) -> bool {
//     query.iter().any(|ekf| ekf.timer.just_finished())
// }

// /// PREDICT step. Runs only when the `should_run_ekf_system` gate is open.
// fn ekf_predict_system(mut query: Query<(Entity, &mut EkfState, &DynamicsModel, &EKF)>) {
//     for (entity, mut _ekf_state, _dynamics, ekf) in query.iter_mut() {
//         // We check the timer here to ensure we only predict for the
//         // specific instances that are ready on this tick.
//         if ekf.timer.just_finished() {
//             println!("[ESTIMATOR] PREDICT step for entity {:?}", entity);
//             // Predict logic would go here...
//         }
//     }
// }

// /// UPDATE step. This system also only runs when the gate is open, ensuring
// /// it processes a full batch of accumulated events.
// fn ekf_update_from_imu(
//     mut agent_query: Query<(Entity, &mut EkfState)>,
//     mut imu_reader: EventReader<ImuData>,
//     // Use the new debug counter resource.
//     mut debug_counter: ResMut<DebugCounter>,
// ) {
//     let events_in_queue: Vec<_> = imu_reader.read().collect();
//     if events_in_queue.is_empty() {
//         return;
//     }

//     // This is our new, reliable debug print.
//     println!(
//         "[DEBUG] EKF Update sees {} events in queue.",
//         events_in_queue.len()
//     );

//     // Batch events by the entity they belong to.
//     let mut events_by_entity: HashMap<Entity, Vec<&&ImuData>> = HashMap::new();
//     for imu_event in &events_in_queue {
//         events_by_entity
//             .entry(imu_event.entity)
//             .or_default()
//             .push(imu_event);
//     }

//     // Iterate over agents with an EKF component.
//     for (agent_entity, mut _ekf_state) in agent_query.iter_mut() {
//         if let Some(events_for_this_agent) = events_by_entity.get(&agent_entity) {
//             let consumed_count = events_for_this_agent.len();
//             debug_counter.imu_consumed_this_tick += consumed_count;

//             println!(
//                 "[ESTIMATOR] UPDATE step for entity {:?}: processing batch of {} messages.",
//                 agent_entity, consumed_count
//             );

//             // Loop through the batch and apply updates...
//             // for imu_event in events_for_this_agent {
//             //     // ... update logic for each message ...
//             // }
//         }
//     }
// }

// /// OUTPUT step. Runs only when the gate is open.
// fn ekf_write_output(mut query: Query<(Entity, &mut EstimatedPose, &EkfState, &EKF)>) {
//     for (entity, mut _est_pose, _ekf_state, ekf) in query.iter_mut() {
//         if ekf.timer.just_finished() {
//             println!("[ESTIMATOR] OUTPUT step for entity {:?}", entity);
//             // Write output logic here...
//         }
//     }
// }

use crate::simulation::core::dynamics::DynamicsModel;
use crate::simulation::core::topics::{EstimatedPose, ImuData, TopicBus, TopicReader};
use bevy::prelude::*;

// --- Plugin Definition ---
pub struct EkfPlugin;

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct EkfSystemSet;

impl Plugin for EkfPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedUpdate, tick_ekf_timers).add_systems(
            FixedUpdate,
            (
                ekf_predict_system,
                // We now have a separate update system for each data type.
                ekf_update_from_imu.after(ekf_predict_system),
                ekf_write_output.after(ekf_update_from_imu),
            )
                .in_set(EkfSystemSet)
                .after(tick_ekf_timers)
                .run_if(should_run_ekf_system),
        );
    }
}

// --- Components ---

// This component holds ALL IMU subscriptions for one EKF instance.
#[derive(Component, Default)]
pub struct ImuSubscriptions {
    // It's just a vector of the generic TopicReaders.
    pub readers: Vec<TopicReader<ImuData>>,
}

// You would have another one for GPS:
// #[derive(Component, Default)]
// pub struct GpsSubscriptions {
//     pub readers: Vec<TopicReader<GpsData>>,
// }

#[derive(Component)]
pub struct EKF {
    pub timer: Timer,
}
#[derive(Component, Default)]
pub struct EkfState {/* ... */}

// --- Systems ---

/// System 1: Tick all EKF timers.
fn tick_ekf_timers(mut query: Query<&mut EKF>, time: Res<Time<Fixed>>) {
    for mut ekf in query.iter_mut() {
        ekf.timer.tick(time.delta());
    }
}

/// System 2: Read-only run condition.
fn should_run_ekf_system(query: Query<&EKF>) -> bool {
    let should_run = query.iter().any(|ekf| ekf.timer.just_finished());
    if should_run {
        // This print statement is the most important debug tool here.
        // If you never see this, the timers are not being ticked correctly.
        println!("[SCHEDULER] EKF SystemSet should run this frame.");
    }
    should_run
}

/// System 3a: PREDICT step.
/// It no longer needs to check the timer, as the run condition already did.
fn ekf_predict_system(mut query: Query<(&mut EkfState, &DynamicsModel, &EKF)>) {
    println!("[ESTIMATOR] EKF Predict running...");
    for (mut _ekf_state, _dynamics, ekf) in query.iter_mut() {
        // We can optionally check again if we want to only predict for the *specific*
        // EKF that is ready, in a multi-agent scenario with different rates.
        if ekf.timer.just_finished() {
            // Predict logic...
        }
    }
}

/// System 3b: UPDATE step. This runs every time the set runs.
fn ekf_update_from_imu(
    topic_bus: Res<TopicBus>,
    mut query: Query<(&mut EkfState, &mut ImuSubscriptions)>,
) {
    println!("[ESTIMATOR] EKF Update running...");
    for (mut _ekf_state, mut subscriptions) in query.iter_mut() {
        for reader in &mut subscriptions.readers {
            let topic_name = reader.topic_name.clone();
            if let Some(topic) = topic_bus.get_topic::<ImuData>(&topic_name) {
                let messages: Vec<_> = reader.read(topic).collect();
                if !messages.is_empty() {
                    println!(
                        "EKF consumed {} IMU messages from topic '{}'",
                        messages.len(),
                        topic_name
                    );
                }
            }
        }
    }
}

fn ekf_write_output(mut query: Query<(&mut EstimatedPose, &EkfState, &EKF)>) {
    for (mut _public_pose, _private_state, ekf) in query.iter_mut() {
        if ekf.timer.just_finished() {
            // Write output logic...
        }
    }
}
