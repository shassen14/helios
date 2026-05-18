use bevy::prelude::*;
use nalgebra::Vector3;
use rand_distr::{Distribution, Normal};
use std::time::Duration;

use crate::prelude::*;
use crate::simulation::core::{app_state::SimulationSet, prng::SimulationRng, transforms::EnuVector};
use crate::simulation::plugins::autonomy::components::{
    AutonomyPipelineComponent, SensorPublishChannel,
};

use helios_core::data::primitives::{FrameHandle, MonotonicTime};
use helios_core::data::sensor::{GpsPosition, SensorReading};
use helios_runtime::pipeline::node::HOST_PRODUCER_ID;
use helios_runtime::port::ChannelKey;
use helios_runtime::stamped::{Health, Stamped};

// =========================================================================
// == GPS Components & Plugin ==
// =========================================================================

#[derive(Component)]
pub struct Gps {
    pub timer: Timer,
    noise_dist: Normal<f64>,
}

pub struct GpsPlugin;

impl Plugin for GpsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_gps_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            gps_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

fn spawn_gps_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        for (sensor_name, sensor_config) in &request.0.sensors {
            if let SensorConfig::Gps(gps_config) = sensor_config {
                info!(
                    "  -> Spawning GPS '{}' as child of agent '{}'",
                    &sensor_name,
                    agent_name.as_str()
                );

                let stddev = gps_config.noise_stddev[0] as f64;
                if stddev <= 0.0 {
                    error!(
                        "GPS sensor '{}' has invalid noise_stddev={}: must be > 0. Skipping sensor.",
                        sensor_name, stddev
                    );
                    continue;
                }

                let sensor_entity = commands
                    .spawn((
                        Name::new(format!("{}/{}", agent_name.as_str(), gps_config.name)),
                        Gps {
                            timer: Timer::new(
                                Duration::from_secs_f32(1.0 / gps_config.rate),
                                TimerMode::Repeating,
                            ),
                            noise_dist: Normal::new(0.0, stddev).unwrap(),
                        },
                        SensorPublishChannel(gps_config.channel.clone()),
                        TrackedFrame,
                        gps_config.get_relative_pose().to_bevy_local_transform(),
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

fn gps_sensor_system(
    time: Res<Time>,
    mut rng: ResMut<SimulationRng>,
    mut sensor_query: Query<(Entity, &mut Gps, &GlobalTransform, &SensorPublishChannel, &ChildOf)>,
    pipeline_query: Query<&AutonomyPipelineComponent>,
) {
    let elapsed = time.elapsed_secs_f64();
    let dt = time.delta();

    for (sensor_entity, mut gps, sensor_global_transform, channel, parent) in &mut sensor_query {
        gps.timer.tick(dt);
        if !gps.timer.just_finished() {
            continue;
        }

        let Ok(pipeline_comp) = pipeline_query.get(parent.parent()) else {
            continue;
        };

        let true_position_bevy = sensor_global_transform.translation();
        let true_position_enu = EnuVector::from(true_position_bevy).0;

        let noisy_position = Vector3::new(
            true_position_enu.x + gps.noise_dist.sample(&mut rng.0),
            true_position_enu.y + gps.noise_dist.sample(&mut rng.0),
            true_position_enu.z + gps.noise_dist.sample(&mut rng.0),
        );

        let reading = SensorReading {
            sensor_handle: FrameHandle::from_entity(sensor_entity),
            timestamp: MonotonicTime(elapsed),
            data: GpsPosition { position: noisy_position },
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
                ChannelKey::named::<Vec<SensorReading<GpsPosition>>>(channel.0.as_str()),
                stamped,
            )
            .ok();
    }
}
