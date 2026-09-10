use crate::brain_bridge::components::SensorPublishChannel;
use crate::config::structs::SensorConfig;
use crate::core::app_state::SimulationSet;
use crate::core::prng::{MasterSeed, SensorRng};
use crate::core::transforms::{freevector_bevy_to_vec3, ToBevy};
use crate::prelude::*;

use helios_core::data::envelope::SensorReading;
use helios_core::data::primitives::{FrameHandle, MonotonicTime};
use helios_core::frames::conventions::Flu;
use helios_core::frames::quantities::FreeVector;
use helios_core::frames::transforms::Convention;
use helios_core::sensors::{lidar::LidarModel, RayHit, RaycastingOutput, RaycastingSensorModel};

use avian3d::prelude::{SpatialQuery, SpatialQueryFilter};
use std::time::Duration;

// =========================================================================
// == Components & Plugin ==
// =========================================================================

#[derive(Component)]
pub struct RaycastingSensor {
    pub timer: Timer,
    pub model: Box<dyn RaycastingSensorModel>,
}

pub struct RaycastingSensorPlugin;

impl Plugin for RaycastingSensorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_raycasting_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            raycasting_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

fn spawn_raycasting_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
    master_seed: Res<MasterSeed>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        for (sensor_name, sensor_config) in &request.0.sensors {
            if let SensorConfig::Lidar(lidar_config) = sensor_config {
                info!(
                    "  -> Spawning LiDAR '{}' as RaycastingSensor for agent '{}' with rate of {:.1} Hz",
                    sensor_name,
                    agent_name.as_str(),
                    lidar_config.get_rate()
                );

                let ring_elevations: Vec<f64> = lidar_config
                    .ring_elevations
                    .iter()
                    .map(|deg| deg.to_radians())
                    .collect();

                let Some(model) = LidarModel::new(
                    lidar_config.azimuth_fov.to_radians(),
                    lidar_config.azimuth_beams,
                    ring_elevations,
                    lidar_config.sweep_period,
                    lidar_config.max_range,
                    lidar_config.range_noise_stddev,
                    lidar_config.angular_noise_stddev,
                ) else {
                    error!(
                        "LiDAR '{}' has an unusable configuration (range_noise_stddev={}, angular_noise_stddev={}, azimuth_beams={}, ring_elevations={}): noise stddevs must be > 0 and there must be at least one azimuth beam and one ring. Skipping sensor.",
                        sensor_name,
                        lidar_config.range_noise_stddev,
                        lidar_config.angular_noise_stddev,
                        lidar_config.azimuth_beams,
                        lidar_config.ring_elevations.len(),
                    );
                    continue;
                };
                let core_model: Box<dyn RaycastingSensorModel> = Box::new(model);

                let mut sensor_entity_commands = commands.spawn_empty();
                let sensor_entity = sensor_entity_commands.id();

                let sensor_label = format!("{}/{}", agent_name.as_str(), sensor_name);
                let sensor_rng = SensorRng::from_sensor(master_seed.0, &sensor_label);

                sensor_entity_commands.insert((
                    Name::new(sensor_label),
                    SensorPublishChannel(lidar_config.get_channel().to_string()),
                    RaycastingSensor {
                        timer: Timer::new(
                            Duration::from_secs_f64(1.0 / lidar_config.get_rate()),
                            TimerMode::Repeating,
                        ),
                        model: core_model,
                    },
                    sensor_rng,
                    TrackedFrame(Convention::Flu),
                    lidar_config.get_relative_pose().to_bevy_local_transform(),
                ));

                commands.entity(agent_entity).add_child(sensor_entity);
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

fn raycasting_sensor_system(
    time: Res<Time>,
    spatial_query: SpatialQuery,
    mut sensor_query: Query<(
        Entity,
        &mut RaycastingSensor,
        &mut SensorRng,
        &SensorPublishChannel,
        &GlobalTransform,
        &ChildOf,
    )>,
    mut publisher: SensorPublisher,
) {
    let _span = tracing::trace_span!("sim.sensor.publish", sensor = "raycasting").entered();
    let elapsed = time.elapsed_secs_f64();
    let dt = time.delta();

    for (sensor_entity, mut sensor, mut rng, sensor_publish_channel, sensor_transform, parent) in
        &mut sensor_query
    {
        sensor.timer.tick(dt);
        if !sensor.timer.just_finished() {
            continue;
        }

        let local_rays = sensor.model.generate_rays();
        let mut hits: Vec<RayHit> = Vec::with_capacity(local_rays.len());
        let sensor_origin = sensor_transform.translation();
        let sensor_rotation = sensor_transform.rotation();
        let max_toi = sensor.model.get_max_range();

        let filter = SpatialQueryFilter::from_excluded_entities([parent.parent()]);

        for ray in local_rays {
            let bevy_local_dir = FreeVector::<Flu>::from_raw(ray.direction).to_bevy();

            let world_direction: Vec3 = sensor_rotation * freevector_bevy_to_vec3(bevy_local_dir);

            if let Ok(dir) = Dir3::new(world_direction) {
                if let Some(hit) =
                    spatial_query.cast_ray(sensor_origin, dir, max_toi, true, &filter)
                {
                    hits.push(RayHit {
                        ray_id: ray.id,
                        distance: hit.distance,
                    });
                }
            }
        }

        let output = sensor.model.process_hits(&hits, &mut rng.0);

        let RaycastingOutput::PointCloud(point_cloud) = output;

        let reading = SensorReading {
            sensor_handle: FrameHandle::from_entity(sensor_entity),
            timestamp: MonotonicTime(elapsed),
            data: point_cloud,
        };

        publisher.publish(
            parent.parent(),
            sensor_publish_channel.0.as_str(),
            vec![reading],
        );
    }
}
