use crate::simulation::core::topics::{GroundTruthState, ImuData, TopicBus};
use bevy::prelude::*;

// --- Plugin Definition ---
pub struct ImuPlugin;

// #[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
// pub enum ImuSystemSet {
//     // Add `pub`
//     Tick,
//     Main,
// }

// impl Plugin for ImuPlugin {
//     fn build(&self, app: &mut App) {
//         app.add_event::<ImuData>()
//             // Now we use the public enum to label our systems.
//             .add_systems(FixedUpdate, tick_imu_timers.in_set(ImuSystemSet::Tick))
//             .add_systems(
//                 FixedUpdate,
//                 imu_sensor_system
//                     .in_set(ImuSystemSet::Main)
//                     .after(ImuSystemSet::Tick)
//                     .run_if(should_run_imu_system),
//             );
//     }
// }

// // --- Components ---
// #[derive(Component)]
// pub struct Imu {
//     pub name: String,
//     pub noise_stddev_accel: f64,
//     pub noise_stddev_gyro: f64,
//     // The timer is still here, but it will be mutated by a dedicated system.
//     pub timer: Timer,
// }

// // --- Systems ---

// /// SYSTEM 1: Ticks all IMU timers. Runs every frame.
// /// This system has write access (`&mut Imu`).
// fn tick_imu_timers(mut query: Query<&mut Imu>, time: Res<Time>) {
//     for mut imu in query.iter_mut() {
//         imu.timer.tick(time.delta());
//     }
// }

// /// SYSTEM 2: The Run Condition. Now READ-ONLY.
// /// This system only has read access (`&Imu`). It cannot mutate the timer.
// fn should_run_imu_system(query: Query<&Imu>) -> bool {
//     // Check if any timer has *just finished* on this frame.
//     // The actual ticking was handled by `tick_imu_timers`.
//     query.iter().any(|imu| imu.timer.just_finished())
// }

// /// SYSTEM 3: The Main System. Also READ-ONLY (for the Imu component).
// /// It runs only if the run condition returned true.
// fn imu_sensor_system(
//     // Query is now read-only for `&Imu`.
//     agent_query: Query<(Entity, &Imu, &GroundTruthState)>,
//     mut imu_writer: EventWriter<ImuData>,
//     time: Res<Time>,
//     mut debug_counter: ResMut<DebugCounter>,
// ) {
//     for (entity, imu, ground_truth) in agent_query.iter() {
//         // We check `just_finished` again to process ONLY the ready IMUs.
//         if imu.timer.just_finished() {
//             println!(
//                 "[SENSOR] IMU '{}' on entity {:?} is firing.",
//                 imu.name, entity
//             );

//             let event = ImuData {
//                 entity,
//                 sensor_name: imu.name.clone(),
//                 timestamp: time.elapsed_secs_f64(),
//                 acceleration: ground_truth.linear_velocity, // Simplified
//                 angular_velocity: ground_truth.angular_velocity,
//             };
//             imu_writer.write(event);

//             debug_counter.imu_produced_this_tick += 1;
//         }
//     }
// }

impl Plugin for ImuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedUpdate, tick_imu_timers).add_systems(
            FixedUpdate,
            imu_sensor_system
                .after(tick_imu_timers)
                .run_if(should_run_imu_system),
        );
    }
}

// --- Components ---
#[derive(Component)]
pub struct Imu {
    pub name: String,
    // This is the full topic name this specific IMU publishes to.
    pub topic_to_publish: String,
    pub timer: Timer,
}

// This is the SINGLE component added to an agent. It manages ALL IMUs for that agent.
#[derive(Component, Default)]
pub struct ImuSuite {
    pub imus: Vec<Imu>,
}

// --- Systems ---

// This system ticks ALL IMU timers within ALL suites.
fn tick_imu_timers(mut query: Query<&mut ImuSuite>, time: Res<Time<Fixed>>) {
    for mut suite in query.iter_mut() {
        for imu in &mut suite.imus {
            imu.timer.tick(time.delta());
        }
    }
}

// This run condition checks if ANY IMU in ANY suite is ready.
fn should_run_imu_system(query: Query<&ImuSuite>) -> bool {
    query
        .iter()
        .any(|suite| suite.imus.iter().any(|imu| imu.timer.just_finished()))
}

// The main system iterates through the suites and their IMUs.
fn imu_sensor_system(
    agent_query: Query<(Entity, &ImuSuite, &GroundTruthState)>,
    mut topic_bus: ResMut<TopicBus>,
    time: Res<Time>,
) {
    for (entity, suite, ground_truth) in agent_query.iter() {
        // Loop through each IMU spec in the suite.
        for imu in &suite.imus {
            // Check if THIS specific IMU is ready.
            if imu.timer.just_finished() {
                println!(
                    "[SENSOR] IMU '{}' firing to topic '{}'.",
                    imu.name, imu.topic_to_publish
                );
                let data = ImuData {
                    entity,
                    sensor_name: imu.name.clone(),
                    timestamp: time.elapsed_secs_f64(),
                    acceleration: ground_truth.linear_velocity,
                    angular_velocity: ground_truth.angular_velocity,
                };
                topic_bus.publish(&imu.topic_to_publish, data);
            }
        }
    }
}

// fn imu_sensor_system(
//     agent_query: Query<(Entity, &Imu, &GroundTruthState)>,
//     mut topic_bus: ResMut<TopicBus>, // Get mutable access to the bus
//     time: Res<Time>,
// ) {
//     for (entity, imu, ground_truth) in agent_query.iter() {
//         if imu.timer.just_finished() {
//             println!(
//                 "[SENSOR] IMU '{}' firing to topic '{}'.",
//                 imu.name, imu.topic_to_publish
//             );

//             let data = ImuData {
//                 entity,
//                 sensor_name: imu.name.clone(),
//                 timestamp: time.elapsed_secs_f64(),
//                 acceleration: ground_truth.linear_velocity,
//                 angular_velocity: ground_truth.angular_velocity,
//             };

//             // Publish to the specific topic name associated with this sensor.
//             if !topic_bus.publish(&imu.topic_to_publish, data) {
//                 eprintln!(
//                     "WARN: Failed to publish to topic '{}'. Does it exist?",
//                     imu.topic_to_publish
//                 );
//             }
//         }
//     }
// }
