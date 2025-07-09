// src/simulation/plugins/sensors/imu.rs

use std::time::Duration;

use crate::{
    prelude::{AppState, ImuConfig, SensorConfig},
    simulation::core::{
        abstractions::{Measurement, MeasurementModel},
        app_state::SceneBuildSet,
        frames::{FrameAwareState, FrameId, MeasurementEvent, StateVariable},
        prng::SimulationRng,
        simulation_setup::SpawnAgentConfigRequest,
        topics::GroundTruthState,
    },
};
use avian3d::prelude::Gravity;
use bevy::prelude::*;
use nalgebra::{DMatrix, DVector, Vector3};
use rand_distr::{Distribution, Normal};

const WORLD_MAGNETIC_FIELD_VECTOR: Vector3<f64> = Vector3::new(0.0, 25.0, 0.0); // [East, North, Up] in microteslas

// --- IMU-SPECIFIC COMPONENTS & MODELS ---
#[derive(Component)]
pub struct Imu {
    pub topic_to_publish: String,
    pub timer: Timer,
}

#[derive(Debug)]
pub struct Imu6DofModel {
    agent_entity: Entity,
    sensor_entity: Entity,
    r_matrix: DMatrix<f64>, // 6x6 noise matrix
}

impl Measurement for Imu6DofModel {
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        vec![
            StateVariable::Ax(FrameId::Sensor(self.sensor_entity)),
            StateVariable::Ay(FrameId::Sensor(self.sensor_entity)),
            StateVariable::Az(FrameId::Sensor(self.sensor_entity)),
            StateVariable::Wx(FrameId::Body(self.agent_entity)),
            StateVariable::Wy(FrameId::Body(self.agent_entity)),
            StateVariable::Wz(FrameId::Body(self.agent_entity)),
        ]
    }

    fn get_r(&self) -> &DMatrix<f64> {
        &self.r_matrix
    }

    fn predict_measurement(&self, ekf_state: &FrameAwareState) -> DVector<f64> {
        // A simple model predicts that the measurement is just the current state.
        // A more complex model could account for lever arm effects here.
        let mut z_pred = DVector::zeros(6);
        for (i, var) in self.get_measurement_layout().iter().enumerate() {
            if let Some(idx) = ekf_state.find_idx(var) {
                z_pred[i] = ekf_state.vector[idx];
            }
        }
        z_pred
    }

    fn calculate_jacobian(&self, ekf_state: &FrameAwareState) -> DMatrix<f64> {
        let mut h = DMatrix::zeros(6, ekf_state.dim());
        for (meas_idx, meas_var) in self.get_measurement_layout().iter().enumerate() {
            if let Some(state_idx) = ekf_state.find_idx(meas_var) {
                h[(meas_idx, state_idx)] = 1.0;
            }
        }
        h
    }
}

#[derive(Debug)]
pub struct Imu9DofModel {
    agent_entity: Entity,
    sensor_entity: Entity,
    r_matrix: DMatrix<f64>, // 9x9 noise matrix
    world_magnetic_field: Vector3<f64>,
}

impl Measurement for Imu9DofModel {
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        // A 9-DOF sensor measures 9 things
        vec![
            StateVariable::Ax(FrameId::Sensor(self.sensor_entity)),
            StateVariable::Ay(FrameId::Sensor(self.sensor_entity)),
            StateVariable::Az(FrameId::Sensor(self.sensor_entity)),
            StateVariable::Wx(FrameId::Body(self.agent_entity)),
            StateVariable::Wy(FrameId::Body(self.agent_entity)),
            StateVariable::Wz(FrameId::Body(self.agent_entity)),
            // Note: These should probably be defined in the World frame and rotated
            // into the sensor frame during the prediction step.
            StateVariable::MagX(FrameId::World),
            StateVariable::MagY(FrameId::World),
            StateVariable::MagZ(FrameId::World),
        ]
    }
    fn get_r(&self) -> &DMatrix<f64> {
        &self.r_matrix
    }

    fn predict_measurement(&self, _ekf_state: &FrameAwareState) -> DVector<f64> {
        // A real implementation would get the orientation from the ekf_state,
        // rotate self.world_magnetic_field into the sensor frame, and place it
        // in the last 3 elements of z_pred.
        DVector::zeros(9) // Placeholder
    }

    fn calculate_jacobian(&self, ekf_state: &FrameAwareState) -> DMatrix<f64> {
        // The implementation here would be more complex, as it needs to handle
        // the partial derivatives of the rotation for the magnetometer part.
        DMatrix::zeros(9, ekf_state.dim()) // Placeholder
    }
}
// --- Plugin Definition ---
pub struct ImuPlugin;

impl Plugin for ImuPlugin {
    fn build(&self, app: &mut App) {
        // This single system handles the creation of all IMU sensors.
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_imu_sensors.in_set(SceneBuildSet::ProcessSensors),
        );

        // This is the runtime system that makes the sensors work.
        app.add_systems(
            FixedUpdate,
            imu_sensor_system.run_if(in_state(AppState::Running)),
        );
    }
}

// --- Systems ---

fn spawn_imu_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    // 1. Iterate over each agent that has a spawn request.
    for (agent_entity, request) in &request_query {
        // 2. We are going to add children to this specific agent entity.
        let mut agent_entity_commands = commands.entity(agent_entity);

        // 3. The `with_children` closure gives us a `parent` builder,
        //    which we use to spawn all children for this agent.
        agent_entity_commands.with_children(|parent| {
            // 4. Iterate through the list of sensor configurations from the TOML.
            for sensor_config in &request.0.sensors {
                if let SensorConfig::Imu(imu_config) = sensor_config {
                    // --- This is the key insight ---
                    // We need the child's Entity ID to create the model, but we can't
                    // know it before we spawn. So, we spawn a "placeholder" entity first,
                    // get its ID, and then add the rest of the components.

                    // 5. Spawn a mostly empty entity just to get its ID.
                    //    We can give it a name right away.
                    let mut sensor_entity_commands =
                        parent.spawn(Name::new(format!("Sensor: {}", imu_config.get_name())));
                    let sensor_entity = sensor_entity_commands.id(); // Get the ID!

                    info!(
                        "  -> Spawning IMU '{}' (Entity {:?}) as child of agent {:?}",
                        imu_config.get_name(),
                        sensor_entity,
                        agent_entity
                    );

                    // 6. Now that we have `agent_entity` and `sensor_entity`, we can
                    //    create the concrete MeasurementModel.
                    let model: Box<dyn Measurement> = match imu_config {
                        ImuConfig::SixDof {
                            accel_noise_stddev,
                            gyro_noise_stddev,
                            ..
                        } => {
                            let r_matrix = DMatrix::from_diagonal(&DVector::from_vec(vec![
                                accel_noise_stddev[0] as f64,
                                accel_noise_stddev[1] as f64,
                                accel_noise_stddev[2] as f64,
                                gyro_noise_stddev[0] as f64,
                                gyro_noise_stddev[1] as f64,
                                gyro_noise_stddev[2] as f64,
                            ]));
                            Box::new(Imu6DofModel {
                                agent_entity,
                                sensor_entity,
                                r_matrix,
                            })
                        }
                        ImuConfig::NineDof {
                            accel_noise_stddev,
                            gyro_noise_stddev,
                            mag_noise_stddev,
                            ..
                        } => {
                            // ... logic to build 9-DOF model and its 9x9 R matrix ...
                            let r_matrix = DMatrix::from_diagonal(&DVector::from_vec(vec![
                                accel_noise_stddev[0] as f64,
                                accel_noise_stddev[1] as f64,
                                accel_noise_stddev[2] as f64,
                                gyro_noise_stddev[0] as f64,
                                gyro_noise_stddev[1] as f64,
                                gyro_noise_stddev[2] as f64,
                                mag_noise_stddev[0] as f64,
                                mag_noise_stddev[1] as f64,
                                mag_noise_stddev[2] as f64,
                            ]));
                            Box::new(Imu9DofModel {
                                agent_entity,
                                sensor_entity,
                                r_matrix,
                                world_magnetic_field: WORLD_MAGNETIC_FIELD_VECTOR,
                            })
                        }
                    };

                    // 7. Now we use the `sensor_entity_commands` we got from `parent.spawn()`
                    //    to INSERT the rest of the components onto the child entity.
                    //    This does NOT cause a double-borrow of `commands`.
                    sensor_entity_commands.insert((
                        MeasurementModel(model),
                        Imu {
                            topic_to_publish: format!(
                                "/agent/{}/imu/{}",
                                request.0.name,
                                imu_config.get_name()
                            ),
                            timer: Timer::new(
                                Duration::from_secs_f32(1.0 / imu_config.get_rate()),
                                TimerMode::Repeating,
                            ),
                        },
                        imu_config.get_relative_pose().to_bevy_transform(),
                    ));
                }
            }
        });
    }
}
fn imu_sensor_system(
    // --- Resources ---
    mut measurement_writer: EventWriter<MeasurementEvent>,
    time: Res<Time>,
    gravity: Res<Gravity>,
    mut rng: ResMut<SimulationRng>,

    // --- Queries ---
    // QUERY 1: Find all parent agents that have children and a GroundTruthState.
    parent_query: Query<(Entity, &GroundTruthState, &Children)>,

    // QUERY 2: This query accesses components on the child sensor entities.
    // It gets the Imu for the timer/topic and the sensor's own world transform.
    mut sensor_query: Query<(&mut Imu, &GlobalTransform)>,
) {
    // 1. Iterate over each parent agent.
    for (agent_entity, ground_truth, children) in &parent_query {
        // 2. Iterate through this agent's children.
        for &child_entity in children {
            // 3. Use get_mut to check if this child is an IMU sensor.
            if let Ok((mut imu, sensor_global_transform)) = sensor_query.get_mut(child_entity) {
                imu.timer.tick(time.delta());
                if !imu.timer.just_finished() {
                    continue; // Skip to the next sensor if this one isn't ready.
                }

                // --- 4. PERFORM PHYSICAL CALCULATIONS ---
                // 1. Get the Bevy quaternion
                let (_, bevy_quat, _) = sensor_global_transform.to_scale_rotation_translation();
                // 2. Manually construct the nalgebra quaternion from its components
                let nalgebra_quat =
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        bevy_quat.w as f64,
                        bevy_quat.x as f64,
                        bevy_quat.y as f64,
                        bevy_quat.z as f64,
                    ));
                let sensor_world_rotation = nalgebra_quat;
                let q_body_from_world = sensor_world_rotation.inverse();

                // --- Gyroscope Reading ---
                let perfect_gyro_reading = q_body_from_world * ground_truth.angular_velocity;

                // --- Accelerometer Reading ---
                let world_gravity_vector = Vector3::new(0.0, gravity.0.y as f64, 0.0);
                let proper_acceleration_world =
                    ground_truth.linear_acceleration - world_gravity_vector;
                let perfect_accel_reading = q_body_from_world * proper_acceleration_world;

                // For now, we'll just create a 6-element measurement vector.
                // A more advanced version would check the model type or layout size.
                let mut z = DVector::from_vec(vec![
                    perfect_accel_reading.x,
                    perfect_accel_reading.y,
                    perfect_accel_reading.z,
                    perfect_gyro_reading.x,
                    perfect_gyro_reading.y,
                    perfect_gyro_reading.z,
                ]);

                // --- 5. ADD NOISE (Simplified) ---
                // In a real system, we would query for the MeasurementModel component,
                // get its R matrix, and generate noise from that. For now, we'll use a placeholder.
                // let noise = ...;
                // z += noise;

                // --- 6. SEND THE GENERIC EVENT ---
                measurement_writer.write(MeasurementEvent {
                    agent_entity,
                    sensor_entity: child_entity,
                    z,
                    timestamp: time.elapsed_secs_f64(),
                });
            }
        }
    }
}

// fn spawn_imu_sensors(
//     mut commands: Commands,
//     mut topic_bus: ResMut<TopicBus>,
//     // Query for agents that have a sensor suite request.
//     request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
// ) {
//     for (agent_entity, agent_name, request) in &request_query {
//         // We will add all IMU children to the agent entity.
//         commands.entity(agent_entity).with_children(|parent| {
//             // Iterate through the list of sensor configs from the TOML.
//             for sensor_config in &request.0.sensors {
//                 // This system only cares about the `Imu` variant.
//                 if let SensorConfig::Imu(imu_config) = sensor_config {
//                     info!(
//                         "  -> Spawning IMU '{}' as child of agent '{}'",
//                         imu_config.get_name(),
//                         agent_name
//                     );

//                     // 1. Create the topic for this specific sensor.
//                     let topic_name = format!(
//                         "/agent/{}/imu/{}",
//                         agent_name.as_str(),
//                         imu_config.get_name()
//                     );
//                     topic_bus.create_topic::<ImuData>(
//                         &topic_name,
//                         100,
//                         TopicTag::Imu,
//                         agent_name.as_str().to_string(),
//                     );

//                     // 2. Build the correct ImuModel from the config.
//                     let imu_model = match imu_config {
//                         ImuConfig::SixDof {
//                             accel_noise_stddev,
//                             gyro_noise_stddev,
//                             ..
//                         } => ImuModel::SixDof {
//                             accel_noise_stddev: accel_noise_stddev.map(|v| v as f64),
//                             gyro_noise_stddev: gyro_noise_stddev.map(|v| v as f64),
//                         },
//                         ImuConfig::NineDof {
//                             accel_noise_stddev,
//                             gyro_noise_stddev,
//                             mag_noise_stddev,
//                             ..
//                         } => ImuModel::NineDof {
//                             accel_noise_stddev: accel_noise_stddev.map(|v| v as f64),
//                             gyro_noise_stddev: gyro_noise_stddev.map(|v| v as f64),
//                             mag_noise_stddev: mag_noise_stddev.map(|v| v as f64),
//                         },
//                     };

//                     // 3. Get the sensor's local transform relative to its parent.
//                     let local_transform = imu_config.get_relative_pose().to_bevy_transform();

//                     // 4. Spawn the child entity with all its components.
//                     parent.spawn((
//                         Name::new(format!("Sensor: {}", imu_config.get_name())),
//                         Imu {
//                             name: imu_config.get_name().to_string(),
//                             topic_to_publish: topic_name,
//                             timer: Timer::new(
//                                 Duration::from_secs_f32(1.0 / imu_config.get_rate()),
//                                 TimerMode::Repeating,
//                             ),
//                             model: imu_model,
//                         },
//                         // Attach the local transform. Bevy will automatically compute the GlobalTransform.
//                         local_transform,
//                     ));
//                 }
//             }
//         });
//     }
// }

// fn imu_sensor_system(
//     // --- QUERY 1: The "Parent" Query ---
//     // We start by finding all agents that have children.
//     // We get the data we need from the parent (GroundTruthState).
//     agent_query: Query<(&GroundTruthState, &Children)>,

//     // --- QUERY 2: The "Child" Query ---
//     // This is a separate query that can access any IMU component in the world.
//     // We will use the list of children from the first query to look up entities here.
//     mut imu_query: Query<(&mut Imu, &GlobalTransform)>,

//     // --- Resources ---
//     mut topic_bus: ResMut<TopicBus>,
//     time: Res<Time<Fixed>>,
//     gravity: Res<Gravity>,
//     mut rng: ResMut<SimulationRng>,
// ) {
//     // 1. Iterate over each agent (the parent).
//     for (ground_truth, children) in &agent_query {
//         // `children` is a component containing a list of the agent's child entities.

//         // 2. Iterate through the list of child entities for this specific agent.
//         for &child_entity in children {
//             // `child_entity` is the Entity ID of a sensor.

//             // 3. Try to get the IMU components from this child entity using our second query.
//             // `get_mut` will return Ok if this child is indeed an IMU, and Err otherwise.
//             if let Ok((mut imu, _sensor_global_transform)) = imu_query.get_mut(child_entity) {
//                 // SUCCESS! We have found an IMU child.
//                 // At this point, we have everything we need:
//                 // - `ground_truth` (from the parent agent)
//                 // - `imu` (the mutable data for this specific sensor)
//                 // - `sensor_global_transform` (the world pose of this specific sensor)

//                 imu.timer.tick(time.delta());

//                 if imu.timer.just_finished() {
//                     // The ground truth pose gives us the orientation of the body relative to the world.
//                     // We need the inverse to transform vectors FROM the world frame TO the body frame.
//                     let q_world_to_body = ground_truth.pose.rotation.inverse();

//                     // --- 1. Calculate Angular Velocity (Gyroscope) ---
//                     // A real gyro measures angular velocity in its own local reference frame.
//                     // The physics engine provides it in the world frame, so we must rotate it.
//                     let body_angular_velocity = q_world_to_body * ground_truth.angular_velocity;

//                     // --- 2. Calculate Proper Acceleration (Accelerometer) ---
//                     // An accelerometer measures the combination of kinematic acceleration and gravity.
//                     // This is also measured in the sensor's local reference frame.

//                     // First, transform the world-frame gravity vector into the body's frame.
//                     let world_gravity_vector = Vector3::new(0.0, gravity.0.y as f64, 0.0);
//                     let gravity_in_body_frame = q_world_to_body * world_gravity_vector;

//                     // Second, transform the world-frame kinematic acceleration into the body's frame.
//                     let kinematic_accel_in_body_frame =
//                         q_world_to_body * ground_truth.linear_acceleration;

//                     // Proper Acceleration = a_kinematic - g. This is what the accelerometer feels.
//                     // For example, an object at rest (a=0) on a table feels an upward force, so it
//                     // measures an upward acceleration of +g. Our formula: 0 - (-g) = +g. Correct.
//                     let proper_acceleration = kinematic_accel_in_body_frame - gravity_in_body_frame;

//                     // --- 3. Generate and Apply Noise ---
//                     // We match on the IMU model to get the correct noise parameters.
//                     let (noisy_acceleration, noisy_angular_velocity) = match &imu.model {
//                         ImuModel::SixDof {
//                             accel_noise_stddev,
//                             gyro_noise_stddev,
//                         } => {
//                             // Create Gaussian distributions for the noise.
//                             let accel_dist = Normal::new(0.0, 1.0).unwrap(); // Will be scaled by stddev
//                             let gyro_dist = Normal::new(0.0, 1.0).unwrap();

//                             // Generate noise vectors.
//                             let accel_noise = Vector3::new(
//                                 accel_dist.sample(&mut rng.0) * accel_noise_stddev[0],
//                                 accel_dist.sample(&mut rng.0) * accel_noise_stddev[1],
//                                 accel_dist.sample(&mut rng.0) * accel_noise_stddev[2],
//                             );
//                             let gyro_noise = Vector3::new(
//                                 gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[0],
//                                 gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[1],
//                                 gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[2],
//                             );

//                             // Add noise to the perfect, body-frame values.
//                             (
//                                 proper_acceleration + accel_noise,
//                                 body_angular_velocity + gyro_noise,
//                             )
//                         }
//                         ImuModel::NineDof { .. } => {
//                             // TODO: Implement magnetometer readings and noise for 9-DOF.
//                             // For now, it behaves identically to the 6-DOF model.
//                             // You would add a magnetometer noise model here.
//                             let noisy_accel = proper_acceleration; // Placeholder
//                             let noisy_gyro = body_angular_velocity; // Placeholder
//                             (noisy_accel, noisy_gyro)
//                         }
//                     };

//                     // --- 4. Construct and Publish the Message ---
//                     let data = ImuData {
//                         entity: child_entity,
//                         sensor_name: imu.name.clone(),
//                         timestamp: time.elapsed_secs_f64(),
//                         acceleration: noisy_acceleration,
//                         angular_velocity: noisy_angular_velocity,
//                     };

//                     // println!("{:?}", data);

//                     topic_bus.publish(&imu.topic_to_publish, data);
//                 }
//             }
//         }
//     }
// }
