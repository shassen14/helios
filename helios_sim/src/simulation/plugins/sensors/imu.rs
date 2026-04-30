// helios_sim/src/simulation/plugins/sensors/imu.rs
use avian3d::prelude::Gravity;
use bevy::prelude::*;
use nalgebra::{DMatrix, Vector3};
use rand_distr::{Distribution, Normal};
use std::time::Duration;

// --- Simulation Crate Imports ---
use crate::prelude::*;
use crate::simulation::core::transforms::EnuBodyPose;
use crate::simulation::core::{
    app_state::SimulationSet, components::GroundTruthState, events::BevyMeasurementMessage,
    prng::SimulationRng,
};

// --- Core Library Imports ---
use helios_core::{
    messages::{MeasurementData, MeasurementMessage},
    models::estimation::measurement::{
        accelerometer::AccelerometerModel, gyroscope::GyroscopeModel,
    },
    sensor_data,
    types::FrameHandle,
};

// =========================================================================
// == IMU Components & Plugin ==
// =========================================================================

/// A Bevy component on the IMU coordinator entity.
///
/// Holds the shared timer and per-axis noise distributions for both the
/// accelerometer and gyroscope sub-entities that this IMU chip produces.
/// The sub-entity handles are stored here so the runtime system can emit
/// correctly-addressed messages without an extra query.
#[derive(Component)]
pub struct Imu {
    pub timer: Timer,
    /// Entity that holds the `AccelerometerModel` (child of the same agent).
    pub accel_entity: Entity,
    /// Entity that holds the `GyroscopeModel` (child of the same agent).
    pub gyro_entity: Entity,
    accel_noise: [Normal<f64>; 3],
    gyro_noise: [Normal<f64>; 3],
}

pub struct ImuPlugin;

impl Plugin for ImuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_imu_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            imu_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

/// Reads the config and spawns IMU sensor entities as children of the appropriate agent.
///
/// Each IMU config produces three child entities of the agent:
/// 1. **Coordinator** — holds the `Imu` component (timer + noise). Its `GlobalTransform`
///    provides the sensor orientation used in the runtime system. No `MeasurementModel`.
/// 2. **Accelerometer entity** — holds `MeasurementModel(AccelerometerModel)` and
///    `TrackedFrame`. Used as `sensor_handle` in `LinearAcceleration` messages.
/// 3. **Gyroscope entity** — holds `MeasurementModel(GyroscopeModel)` and `TrackedFrame`.
///    Used as `sensor_handle` in `AngularVelocity` messages.
///
/// Both measurement entities are direct children of the agent so they appear in
/// the `measurement_models` map built by `spawn_autonomy_pipeline`.
fn spawn_imu_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
    gravity: Res<Gravity>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        for (_sensor_name, sensor_config) in &request.0.sensors {
            if let SensorConfig::Imu(imu_config) = sensor_config {
                info!(
                    "  -> Spawning IMU '{}' as child of agent '{}' with rate of {:.1} Hz",
                    imu_config.get_name(),
                    agent_name.as_str(),
                    imu_config.get_rate()
                );

                let (accel_std, gyro_std) = imu_config.get_noise_stddevs();
                let gravity_magnitude = gravity.0.length() as f64;

                // --- 1. Validate noise stddevs before creating any distributions ---
                let all_stddevs = [
                    ("accel_x", accel_std[0] as f64),
                    ("accel_y", accel_std[1] as f64),
                    ("accel_z", accel_std[2] as f64),
                    ("gyro_x", gyro_std[0] as f64),
                    ("gyro_y", gyro_std[1] as f64),
                    ("gyro_z", gyro_std[2] as f64),
                ];
                let noise_valid = all_stddevs.iter().all(|(axis, std)| {
                    if *std <= 0.0 {
                        error!(
                            "IMU '{}' has invalid {} noise_stddev={}: must be > 0. Skipping sensor.",
                            imu_config.get_name(), axis, std
                        );
                        false
                    } else {
                        true
                    }
                });
                if !noise_valid {
                    continue;
                }

                let agent_handle = FrameHandle::from_entity(agent_entity);
                let sensor_pose = imu_config.get_relative_pose();

                // --- 2. Pre-allocate entity IDs so models can reference them ---
                let imu_entity = commands.spawn_empty().id();
                let accel_entity = commands.spawn_empty().id();
                let gyro_entity = commands.spawn_empty().id();

                // --- 3. Build separate 3×3 R matrices ---
                let accel_r = DMatrix::from_diagonal(&nalgebra::DVector::from_vec(vec![
                    (accel_std[0] as f64).powi(2),
                    (accel_std[1] as f64).powi(2),
                    (accel_std[2] as f64).powi(2),
                ]));
                let gyro_r = DMatrix::from_diagonal(&nalgebra::DVector::from_vec(vec![
                    (gyro_std[0] as f64).powi(2),
                    (gyro_std[1] as f64).powi(2),
                    (gyro_std[2] as f64).powi(2),
                ]));

                // --- 4. Build the helios_core models ---
                let accel_model = AccelerometerModel {
                    agent_handle,
                    sensor_handle: FrameHandle::from_entity(accel_entity),
                    r_matrix: accel_r,
                    gravity_magnitude,
                };
                let gyro_model = GyroscopeModel {
                    agent_handle,
                    sensor_handle: FrameHandle::from_entity(gyro_entity),
                    r_matrix: gyro_r,
                };

                // --- 5. IMU coordinator: timer + noise, provides GlobalTransform ---
                commands.entity(imu_entity).insert((
                    Name::new(format!("{}/{}", agent_name.as_str(), imu_config.get_name())),
                    Imu {
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / imu_config.get_rate()),
                            TimerMode::Repeating,
                        ),
                        accel_entity,
                        gyro_entity,
                        accel_noise: [
                            Normal::new(0.0, accel_std[0] as f64).unwrap(),
                            Normal::new(0.0, accel_std[1] as f64).unwrap(),
                            Normal::new(0.0, accel_std[2] as f64).unwrap(),
                        ],
                        gyro_noise: [
                            Normal::new(0.0, gyro_std[0] as f64).unwrap(),
                            Normal::new(0.0, gyro_std[1] as f64).unwrap(),
                            Normal::new(0.0, gyro_std[2] as f64).unwrap(),
                        ],
                    },
                    TrackedFrame,
                    sensor_pose.to_bevy_local_transform(),
                ));

                // --- 6. Accelerometer measurement entity ---
                commands.entity(accel_entity).insert((
                    Name::new(format!(
                        "{}/{}/accelerometer",
                        agent_name.as_str(),
                        imu_config.get_name()
                    )),
                    MeasurementModel(Box::new(accel_model)),
                    TrackedFrame,
                    sensor_pose.to_bevy_local_transform(),
                ));

                // --- 7. Gyroscope measurement entity ---
                commands.entity(gyro_entity).insert((
                    Name::new(format!(
                        "{}/{}/gyroscope",
                        agent_name.as_str(),
                        imu_config.get_name()
                    )),
                    MeasurementModel(Box::new(gyro_model)),
                    TrackedFrame,
                    sensor_pose.to_bevy_local_transform(),
                ));

                // --- 8. All three are direct children of the agent ---
                commands
                    .entity(agent_entity)
                    .add_child(imu_entity)
                    .add_child(accel_entity)
                    .add_child(gyro_entity);
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

/// Runs every frame to simulate IMU physics and emit `BevyMeasurementMessage` events.
///
/// Emits two events per tick per IMU:
/// - `LinearAcceleration` addressed to the accelerometer entity handle.
/// - `AngularVelocity` addressed to the gyroscope entity handle.
fn imu_sensor_system(
    mut measurement_writer: EventWriter<BevyMeasurementMessage>,
    time: Res<Time>,
    mut rng: ResMut<SimulationRng>,
    gravity: Res<Gravity>,
    parent_query: Query<(Entity, &GroundTruthState, &Children)>,
    mut sensor_query: Query<(&mut Imu, &GlobalTransform)>,
) {
    let dt = time.delta();

    for (agent_entity, ground_truth, children) in &parent_query {
        for &child_entity in children {
            if let Ok((mut imu, sensor_global_transform)) = sensor_query.get_mut(child_entity) {
                imu.timer.tick(dt);
                if !imu.timer.just_finished() {
                    continue;
                }

                // --- 1. Get Sensor Orientation in ENU Frame ---
                let sensor_pose_enu = EnuBodyPose::from(sensor_global_transform).0;
                let q_sensor_from_world = sensor_pose_enu.rotation.inverse();

                // --- 2. Calculate Proper Acceleration ---
                let coord_accel_world_enu = ground_truth.linear_acceleration;
                let gravity_world_enu = Vector3::new(0.0, 0.0, gravity.0.y as f64);
                let proper_accel_world_enu = coord_accel_world_enu - gravity_world_enu;

                // --- 3. Express Vectors in Sensor Frame ---
                let perfect_accel_sensor = q_sensor_from_world * proper_accel_world_enu;
                let perfect_gyro_sensor = q_sensor_from_world * ground_truth.angular_velocity;

                // --- 4. Apply Noise ---
                let noisy_accel = Vector3::new(
                    perfect_accel_sensor.x + imu.accel_noise[0].sample(&mut rng.0),
                    perfect_accel_sensor.y + imu.accel_noise[1].sample(&mut rng.0),
                    perfect_accel_sensor.z + imu.accel_noise[2].sample(&mut rng.0),
                );
                let noisy_gyro = Vector3::new(
                    perfect_gyro_sensor.x + imu.gyro_noise[0].sample(&mut rng.0),
                    perfect_gyro_sensor.y + imu.gyro_noise[1].sample(&mut rng.0),
                    perfect_gyro_sensor.z + imu.gyro_noise[2].sample(&mut rng.0),
                );

                let agent_handle = FrameHandle::from_entity(agent_entity);
                let timestamp = time.elapsed_secs_f64();

                // --- 5. Emit LinearAcceleration message ---
                measurement_writer.write(BevyMeasurementMessage(MeasurementMessage {
                    agent_handle,
                    sensor_handle: FrameHandle::from_entity(imu.accel_entity),
                    timestamp,
                    data: MeasurementData::LinearAcceleration(sensor_data::LinearAcceleration3D {
                        value: noisy_accel,
                    }),
                }));

                // --- 6. Emit AngularVelocity message ---
                measurement_writer.write(BevyMeasurementMessage(MeasurementMessage {
                    agent_handle,
                    sensor_handle: FrameHandle::from_entity(imu.gyro_entity),
                    timestamp,
                    data: MeasurementData::AngularVelocity(sensor_data::AngularVelocity3D {
                        value: noisy_gyro,
                    }),
                }));
            }
        }
    }
}
