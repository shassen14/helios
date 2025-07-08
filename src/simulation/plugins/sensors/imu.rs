// src/simulation/plugins/sensors/imu.rs

use crate::simulation::core::{
    prng::SimulationRng,
    topics::{GroundTruthState, ImuData, TopicBus},
};
use avian3d::prelude::Gravity;
use bevy::prelude::*;
use nalgebra::Vector3;
use rand_distr::{Distribution, Normal};

// --- Plugin Definition ---
pub struct ImuPlugin;

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

// Let's create an enum that mirrors the config structure.
// This will be the data stored in our component.
#[derive(Debug, Clone)]
pub enum ImuModel {
    SixDof {
        accel_noise_stddev: [f64; 3],
        gyro_noise_stddev: [f64; 3],
    },
    NineDof {
        accel_noise_stddev: [f64; 3],
        gyro_noise_stddev: [f64; 3],
        mag_noise_stddev: [f64; 3],
    },
}

#[derive(Component)]
pub struct Imu {
    pub name: String,
    pub topic_to_publish: String,
    pub timer: Timer,
    pub model: ImuModel,
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

pub fn imu_sensor_system(
    agent_query: Query<(Entity, &ImuSuite, &GroundTruthState)>,
    mut topic_bus: ResMut<TopicBus>,
    time: Res<Time>,
    gravity: Res<Gravity>,
    mut rng: ResMut<SimulationRng>,
) {
    for (entity, suite, ground_truth) in agent_query.iter() {
        // Loop through each IMU spec in the suite for this agent.
        for imu in &suite.imus {
            // Check if THIS specific IMU's timer has finished.
            // This allows for multiple IMUs on one agent running at different rates.
            if imu.timer.just_finished() {
                // The ground truth pose gives us the orientation of the body relative to the world.
                // We need the inverse to transform vectors FROM the world frame TO the body frame.
                let q_world_to_body = ground_truth.pose.rotation.inverse();

                // --- 1. Calculate Angular Velocity (Gyroscope) ---
                // A real gyro measures angular velocity in its own local reference frame.
                // The physics engine provides it in the world frame, so we must rotate it.
                let body_angular_velocity = q_world_to_body * ground_truth.angular_velocity;

                // --- 2. Calculate Proper Acceleration (Accelerometer) ---
                // An accelerometer measures the combination of kinematic acceleration and gravity.
                // This is also measured in the sensor's local reference frame.

                // First, transform the world-frame gravity vector into the body's frame.
                let world_gravity_vector = Vector3::new(0.0, gravity.0.y as f64, 0.0);
                let gravity_in_body_frame = q_world_to_body * world_gravity_vector;

                // Second, transform the world-frame kinematic acceleration into the body's frame.
                let kinematic_accel_in_body_frame =
                    q_world_to_body * ground_truth.linear_acceleration;

                // Proper Acceleration = a_kinematic - g. This is what the accelerometer feels.
                // For example, an object at rest (a=0) on a table feels an upward force, so it
                // measures an upward acceleration of +g. Our formula: 0 - (-g) = +g. Correct.
                let proper_acceleration = kinematic_accel_in_body_frame - gravity_in_body_frame;

                // --- 3. Generate and Apply Noise ---
                // We match on the IMU model to get the correct noise parameters.
                let (noisy_acceleration, noisy_angular_velocity) = match &imu.model {
                    ImuModel::SixDof {
                        accel_noise_stddev,
                        gyro_noise_stddev,
                    } => {
                        // Create Gaussian distributions for the noise.
                        let accel_dist = Normal::new(0.0, 1.0).unwrap(); // Will be scaled by stddev
                        let gyro_dist = Normal::new(0.0, 1.0).unwrap();

                        // Generate noise vectors.
                        let accel_noise = Vector3::new(
                            accel_dist.sample(&mut rng.0) * accel_noise_stddev[0],
                            accel_dist.sample(&mut rng.0) * accel_noise_stddev[1],
                            accel_dist.sample(&mut rng.0) * accel_noise_stddev[2],
                        );
                        let gyro_noise = Vector3::new(
                            gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[0],
                            gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[1],
                            gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[2],
                        );

                        // Add noise to the perfect, body-frame values.
                        (
                            proper_acceleration + accel_noise,
                            body_angular_velocity + gyro_noise,
                        )
                    }
                    ImuModel::NineDof { .. } => {
                        // TODO: Implement magnetometer readings and noise for 9-DOF.
                        // For now, it behaves identically to the 6-DOF model.
                        // You would add a magnetometer noise model here.
                        let noisy_accel = proper_acceleration; // Placeholder
                        let noisy_gyro = body_angular_velocity; // Placeholder
                        (noisy_accel, noisy_gyro)
                    }
                };

                // --- 4. Construct and Publish the Message ---
                let data = ImuData {
                    entity,
                    sensor_name: imu.name.clone(),
                    timestamp: time.elapsed_secs_f64(),
                    acceleration: noisy_acceleration,
                    angular_velocity: noisy_angular_velocity,
                };

                // println!("{:?}", data);

                topic_bus.publish(&imu.topic_to_publish, data);
            }
        }
    }
}
