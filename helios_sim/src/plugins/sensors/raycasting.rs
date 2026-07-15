// helios_sim/src/simulation/plugins/sensors/raycasting.rs
use avian3d::prelude::{SpatialQuery, SpatialQueryFilter};
use bevy::prelude::*;
use std::time::Duration;

use crate::brain_bridge::components::{AutonomyPipelineComponent, SensorPublishChannel};
use crate::config::structs::{LidarConfig, SensorConfig};
use crate::core::transforms::FluVector;
use crate::core::{app_state::SimulationSet, prng::SimulationRng};
use crate::prelude::*;

use helios_core::data::envelope::SensorReading;
use helios_core::data::primitives::{FrameHandle, MonotonicTime};
use helios_core::data::sensor::PointCloud2D;
use helios_core::sensors::{
    lidar_2d::Lidar2DModel, RayHit, RaycastingOutput, RaycastingSensorModel,
};
use helios_runtime::pipeline::node::HOST_PRODUCER_ID;
use helios_runtime::port::SensorChannel;
use helios_runtime::stamped::{Health, Stamped};

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

                let core_model: Box<dyn RaycastingSensorModel> = match lidar_config {
                    LidarConfig::Lidar2D {
                        max_range,
                        horizontal_fov,
                        horizontal_beams,
                        range_noise_stddev,
                        ..
                    } => {
                        let angular_noise_stddev_deg = 0.1_f32;
                        let Some(model) = Lidar2DModel::new(
                            *max_range,
                            *horizontal_fov,
                            *horizontal_beams,
                            *range_noise_stddev,
                            angular_noise_stddev_deg,
                        ) else {
                            error!(
                                "LiDAR '{}' has invalid noise parameters (range_noise_stddev={}, angular_noise_stddev_deg={}): both must be > 0. Skipping sensor.",
                                sensor_name, range_noise_stddev, angular_noise_stddev_deg
                            );
                            continue;
                        };
                        Box::new(model)
                    }
                    LidarConfig::Lidar3D { .. } => {
                        unimplemented!("Lidar3D spawning not yet implemented.");
                    }
                };

                let mut sensor_entity_commands = commands.spawn_empty();
                let sensor_entity = sensor_entity_commands.id();

                sensor_entity_commands.insert((
                    Name::new(format!("{}/{}", agent_name.as_str(), sensor_name)),
                    RaycastingSensor {
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / lidar_config.get_rate()),
                            TimerMode::Repeating,
                        ),
                        model: core_model,
                    },
                    SensorPublishChannel(lidar_config.get_channel().to_string()),
                    TrackedFrame,
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
    mut rng: ResMut<SimulationRng>,
    spatial_query: SpatialQuery,
    mut sensor_query: Query<(Entity, &mut RaycastingSensor, &GlobalTransform, &ChildOf)>,
    pipeline_query: Query<&AutonomyPipelineComponent>,
) {
    let _span = tracing::trace_span!("sim.sensor.publish", sensor = "raycasting").entered();
    let elapsed = time.elapsed_secs_f64();
    let dt = time.delta();

    for (sensor_entity, mut sensor, sensor_transform, parent) in &mut sensor_query {
        sensor.timer.tick(dt);
        if !sensor.timer.just_finished() {
            continue;
        }

        let Ok(pipeline_comp) = pipeline_query.get(parent.parent()) else {
            continue;
        };

        let local_rays = sensor.model.generate_rays();
        let mut hits: Vec<RayHit> = Vec::with_capacity(local_rays.len());
        let sensor_origin = sensor_transform.translation();
        let sensor_rotation = sensor_transform.rotation();
        let max_toi = sensor.model.get_max_range();

        let filter = SpatialQueryFilter::from_excluded_entities([parent.parent()]);

        for ray in local_rays {
            let bevy_sensor_local_dir = Vec3::from(FluVector(nalgebra::Vector3::new(
                ray.direction.x,
                ray.direction.y,
                ray.direction.z,
            )));
            let world_direction: Vec3 = sensor_rotation * bevy_sensor_local_dir;

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

        let point_cloud = match output {
            RaycastingOutput::PointCloud2D(cloud) => cloud,
            RaycastingOutput::PointCloud3D(_) => continue,
        };

        let reading = SensorReading {
            sensor_handle: FrameHandle::from_entity(sensor_entity),
            timestamp: MonotonicTime(elapsed),
            data: point_cloud,
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
                SensorChannel::of::<Vec<SensorReading<PointCloud2D>>>().into(),
                stamped,
            )
            .ok();
    }
}
