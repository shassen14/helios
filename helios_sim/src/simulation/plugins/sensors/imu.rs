// helios_sim/src/simulation/plugins/sensors/imu.rs
use avian3d::prelude::Gravity;
use bevy::prelude::*;
use nalgebra::Vector3;
use rand_distr::{Distribution, Normal};
use std::time::Duration;

use crate::prelude::*;
use crate::simulation::core::transforms::EnuBodyPose;
use crate::simulation::core::{
    app_state::SimulationSet, components::GroundTruthState, prng::SimulationRng,
};
use crate::simulation::plugins::autonomy::components::AutonomyPipelineComponent;

use helios_core::data::primitives::{FrameHandle, MonotonicTime};
use helios_core::data::sensor::{AngularVelocity3D, LinearAcceleration3D, SensorReading};
use helios_runtime::pipeline::node::HOST_PRODUCER_ID;
use helios_runtime::port::SensorChannel;
use helios_runtime::stamped::{Health, Stamped};

// =========================================================================
// == IMU Components & Plugin ==
// =========================================================================

#[derive(Component)]
pub struct Imu {
    pub timer: Timer,
    pub accel_entity: Entity,
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

fn spawn_imu_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
    _gravity: Res<Gravity>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        for sensor_config in request.0.sensors.values() {
            if let SensorConfig::Imu(imu_config) = sensor_config {
                info!(
                    "  -> Spawning IMU '{}' as child of agent '{}' with rate of {:.1} Hz",
                    imu_config.get_name(),
                    agent_name.as_str(),
                    imu_config.get_rate()
                );

                let (accel_std, gyro_std) = imu_config.get_noise_stddevs();

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

                let sensor_pose = imu_config.get_relative_pose();

                let imu_entity = commands.spawn_empty().id();
                let accel_entity = commands.spawn_empty().id();
                let gyro_entity = commands.spawn_empty().id();

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

                commands.entity(accel_entity).insert((
                    Name::new(format!(
                        "{}/{}/accelerometer",
                        agent_name.as_str(),
                        imu_config.get_name()
                    )),
                    TrackedFrame,
                    sensor_pose.to_bevy_local_transform(),
                ));

                commands.entity(gyro_entity).insert((
                    Name::new(format!(
                        "{}/{}/gyroscope",
                        agent_name.as_str(),
                        imu_config.get_name()
                    )),
                    TrackedFrame,
                    sensor_pose.to_bevy_local_transform(),
                ));

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

fn imu_sensor_system(
    time: Res<Time>,
    mut rng: ResMut<SimulationRng>,
    gravity: Res<Gravity>,
    mut sensor_query: Query<(&mut Imu, &GlobalTransform, &ChildOf)>,
    parent_query: Query<(&GroundTruthState, &AutonomyPipelineComponent)>,
) {
    let _span = tracing::trace_span!("sim.sensor.publish", sensor = "imu").entered();
    let elapsed = time.elapsed_secs_f64();
    let dt = time.delta();

    for (mut imu, sensor_global_transform, parent) in &mut sensor_query {
        imu.timer.tick(dt);
        if !imu.timer.just_finished() {
            continue;
        }

        let Ok((ground_truth, pipeline_comp)) = parent_query.get(parent.parent()) else {
            continue;
        };

        let sensor_pose_enu = EnuBodyPose::from(sensor_global_transform).0;
        let q_sensor_from_world = sensor_pose_enu.rotation.inverse();

        let gravity_world_enu = Vector3::new(0.0, 0.0, gravity.0.y as f64);
        let proper_accel_world_enu = ground_truth.linear_acceleration - gravity_world_enu;

        let perfect_accel_sensor = q_sensor_from_world * proper_accel_world_enu;
        let perfect_gyro_sensor = q_sensor_from_world * ground_truth.angular_velocity;

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

        let accel_reading = SensorReading {
            sensor_handle: FrameHandle::from_entity(imu.accel_entity),
            timestamp: MonotonicTime(elapsed),
            data: LinearAcceleration3D { value: noisy_accel },
        };
        let accel_stamped = Stamped {
            value: vec![accel_reading],
            timestamp: MonotonicTime(elapsed),
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        let gyro_reading = SensorReading {
            sensor_handle: FrameHandle::from_entity(imu.gyro_entity),
            timestamp: MonotonicTime(elapsed),
            data: AngularVelocity3D { value: noisy_gyro },
        };
        let gyro_stamped = Stamped {
            value: vec![gyro_reading],
            timestamp: MonotonicTime(elapsed),
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        pipeline_comp
            .0
            .bus()
            .write(
                SensorChannel::of::<Vec<SensorReading<LinearAcceleration3D>>>().into(),
                accel_stamped,
            )
            .ok();

        pipeline_comp
            .0
            .bus()
            .write(
                SensorChannel::of::<Vec<SensorReading<AngularVelocity3D>>>().into(),
                gyro_stamped,
            )
            .ok();
    }
}
