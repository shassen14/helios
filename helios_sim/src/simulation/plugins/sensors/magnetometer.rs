use bevy::prelude::*;
use nalgebra::Vector3;
use rand_distr::{Distribution, Normal};
use std::time::Duration;

use crate::prelude::*;
use crate::simulation::core::transforms::EnuBodyPose;
use crate::simulation::core::{app_state::SimulationSet, prng::SimulationRng};
use crate::simulation::plugins::autonomy::components::{
    AutonomyPipelineComponent, SensorPublishChannel,
};

use helios_core::data::primitives::{FrameHandle, MonotonicTime};
use helios_core::data::sensor::{MagneticField3D, SensorReading};
use helios_runtime::pipeline::node::HOST_PRODUCER_ID;
use helios_runtime::port::ChannelKey;
use helios_runtime::stamped::{Health, Stamped};

// =========================================================================
// == Magnetometer Components & Plugin ==
// =========================================================================

#[derive(Component)]
pub struct Magnetometer {
    pub timer: Timer,
    noise_dist_x: Normal<f64>,
    noise_dist_y: Normal<f64>,
    noise_dist_z: Normal<f64>,
}

pub struct MagnetometerPlugin;

impl Plugin for MagnetometerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_magnetometer_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            magnetometer_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

fn spawn_magnetometer_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        for (sensor_name, sensor_config) in &request.0.sensors {
            if let SensorConfig::Magnetometer(mag_config) = sensor_config {
                info!(
                    "  -> Spawning Magnetometer '{}' as child of agent '{}' with rate of {:.1} Hz",
                    sensor_name,
                    agent_name.as_str(),
                    mag_config.get_rate()
                );

                let noise_stddevs = [
                    ("x", mag_config.noise_stddev[0] as f64),
                    ("y", mag_config.noise_stddev[1] as f64),
                    ("z", mag_config.noise_stddev[2] as f64),
                ];
                let noise_valid = noise_stddevs.iter().all(|(axis, std)| {
                    if *std <= 0.0 {
                        error!(
                            "Magnetometer '{}' has invalid {} noise_stddev={}: must be > 0. Skipping sensor.",
                            sensor_name, axis, std
                        );
                        false
                    } else {
                        true
                    }
                });
                if !noise_valid {
                    continue;
                }

                let sensor_entity = commands
                    .spawn((
                        Name::new(format!("{}/{}", agent_name.as_str(), mag_config.name)),
                        Magnetometer {
                            timer: Timer::new(
                                Duration::from_secs_f32(1.0 / mag_config.rate),
                                TimerMode::Repeating,
                            ),
                            noise_dist_x: Normal::new(0.0, mag_config.noise_stddev[0] as f64)
                                .unwrap(),
                            noise_dist_y: Normal::new(0.0, mag_config.noise_stddev[1] as f64)
                                .unwrap(),
                            noise_dist_z: Normal::new(0.0, mag_config.noise_stddev[2] as f64)
                                .unwrap(),
                        },
                        SensorPublishChannel(mag_config.channel.clone()),
                        TrackedFrame,
                        mag_config.get_relative_pose().to_bevy_local_transform(),
                    ))
                    .id();

                commands.entity(agent_entity).add_child(sensor_entity);
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

fn magnetometer_sensor_system(
    time: Res<Time>,
    mut rng: ResMut<SimulationRng>,
    mut sensor_query: Query<(
        Entity,
        &mut Magnetometer,
        &GlobalTransform,
        &SensorPublishChannel,
        &ChildOf,
    )>,
    pipeline_query: Query<&AutonomyPipelineComponent>,
) {
    let _span = tracing::trace_span!("sim.sensor.publish", sensor = "magnetometer").entered();
    let elapsed = time.elapsed_secs_f64();
    let dt = time.delta();
    let world_magnetic_field_enu = Vector3::new(0.0, 1.0, 0.0).normalize();

    for (sensor_entity, mut mag, sensor_global_transform, channel, parent) in &mut sensor_query {
        mag.timer.tick(dt);
        if !mag.timer.just_finished() {
            continue;
        }

        let Ok(pipeline_comp) = pipeline_query.get(parent.parent()) else {
            continue;
        };

        let sensor_pose_enu = EnuBodyPose::from(sensor_global_transform).0;
        let q_sensor_from_world = sensor_pose_enu.rotation.inverse();

        let perfect_mag_reading = q_sensor_from_world * world_magnetic_field_enu;

        let noisy_mag_reading = Vector3::new(
            perfect_mag_reading.x + mag.noise_dist_x.sample(&mut rng.0),
            perfect_mag_reading.y + mag.noise_dist_y.sample(&mut rng.0),
            perfect_mag_reading.z + mag.noise_dist_z.sample(&mut rng.0),
        );

        let reading = SensorReading {
            sensor_handle: FrameHandle::from_entity(sensor_entity),
            timestamp: MonotonicTime(elapsed),
            data: MagneticField3D {
                value: noisy_mag_reading,
            },
        };
        let stamped = Stamped {
            value: vec![reading],
            timestamp: MonotonicTime(elapsed),
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        pipeline_comp
            .0
            .bus()
            .write(
                ChannelKey::named::<Vec<SensorReading<MagneticField3D>>>(channel.0.as_str()),
                stamped,
            )
            .ok();
    }
}
