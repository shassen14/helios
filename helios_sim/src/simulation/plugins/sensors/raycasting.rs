// helios_sim/src/simulation/plugins/sensors/raycasting.rs
use avian3d::prelude::{SpatialQuery, SpatialQueryFilter}; // Import the correct types
use bevy::prelude::*;
use std::time::Duration;

// --- Simulation Crate Imports ---
use crate::prelude::*;
use crate::simulation::config::structs::{LidarConfig, SensorConfig};
use crate::simulation::core::{
    app_state::SimulationSet, events::BevyMeasurementMessage, topics::TopicBus,
};
use crate::simulation::plugins::debugging::ShowDebugGizmos;
use std::sync::Arc;

// --- Core Library Imports ---
use helios_core::{
    messages::{MeasurementData, MeasurementMessage},
    models::perception::{
        lidar_2d::Lidar2DModel, // We'll import our concrete models here
        RayHit,
        RaycastingSensorModel,
    },
    types::FrameHandle,
};

// =========================================================================
// == Components & Plugin ==
// =========================================================================

/// A generic component for any sensor that works by raycasting.
#[derive(Component)]
pub struct RaycastingSensor {
    pub timer: Timer,
    /// It holds a boxed trait object of the specific `helios_core` model.
    pub model: Box<dyn RaycastingSensorModel>,
    /// Pre-computed topic name: `/{agent_name}/sensors/{sensor_name}`
    pub topic_name: String,
    /// Cached result of the last scan: (world_direction, draw_length).
    /// draw_length is the hit distance, or max_range if no hit.
    /// Used by the debug gizmo system so rays shorten to their hit points.
    pub last_scan: Vec<(Vec3, f32)>,
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

// This function now mirrors the structure of your `spawn_imu_sensors`
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

                // --- 1. Create the `helios_core` Model ---
                let core_model: Box<dyn RaycastingSensorModel> = match lidar_config {
                    LidarConfig::Lidar2D {
                        max_range,
                        horizontal_fov,
                        horizontal_beams,
                        range_noise_stddev,
                        ..
                    } => Box::new(Lidar2DModel {
                        max_range: *max_range,
                        horizontal_fov_deg: *horizontal_fov,
                        horizontal_beams: *horizontal_beams,
                        range_noise_stddev: *range_noise_stddev,
                        angular_noise_stddev_deg: 0.1,
                    }),
                    LidarConfig::Lidar3D { .. } => {
                        // Future implementation would go here.
                        unimplemented!("Lidar3D spawning not yet implemented.");
                    }
                };

                // --- 2. Spawn the Sensor Entity ---
                let mut sensor_entity_commands = commands.spawn_empty();
                let sensor_entity = sensor_entity_commands.id();

                sensor_entity_commands.insert((
                    Name::new(sensor_name.clone()),
                    RaycastingSensor {
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / lidar_config.get_rate()),
                            TimerMode::Repeating,
                        ),
                        model: core_model,
                        topic_name: format!(
                            "/{}/sensors/{}",
                            agent_name.as_str(),
                            sensor_name
                        ),
                        last_scan: Vec::new(),
                    },
                    TrackedFrame,
                    lidar_config.get_relative_pose().to_bevy_local_transform(),
                ));

                if lidar_config.get_debug_visuals_flag() {
                    sensor_entity_commands.insert(ShowDebugGizmos {});
                }

                commands.entity(agent_entity).add_child(sensor_entity);
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

/// Runs every frame to simulate the raycasting sensor and publish data.
// This function now mirrors the structure of your `imu_sensor_system`.
fn raycasting_sensor_system(
    mut measurement_writer: EventWriter<BevyMeasurementMessage>,
    mut topic_bus: ResMut<TopicBus>,
    time: Res<Time>,
    // --- CORRECTED SYSTEM PARAMETERS ---
    // SpatialQuery is requested directly, not as a resource.
    spatial_query: SpatialQuery,

    // Use the same parent-child query pattern as the IMU.
    parent_query: Query<(Entity, &Children)>,
    mut sensor_query: Query<(Entity, &mut RaycastingSensor, &GlobalTransform)>,
) {
    let dt = time.delta();

    for (agent_entity, children) in &parent_query {
        for &child_entity in children {
            if let Ok((sensor_entity, mut sensor, sensor_transform)) =
                sensor_query.get_mut(child_entity)
            {
                sensor.timer.tick(dt);
                if !sensor.timer.just_finished() {
                    continue;
                }

                let local_rays = sensor.model.generate_rays();
                let mut hits: Vec<RayHit> = Vec::with_capacity(local_rays.len());
                let mut scan_cache: Vec<(Vec3, f32)> = Vec::with_capacity(local_rays.len());
                let sensor_origin = sensor_transform.translation();
                let sensor_rotation = sensor_transform.rotation();
                let max_toi = sensor.model.get_max_range();

                // We create a filter that excludes the sensor's own parent agent.
                let filter = SpatialQueryFilter::from_excluded_entities([agent_entity]);

                for ray in local_rays {
                    // 1. Convert ray direction from sensor FLU frame to Bevy local frame.
                    //    FLU +X (Forward) → Bevy -Z, FLU +Y (Left) → Bevy -X, FLU +Z (Up) → Bevy +Y
                    let bevy_sensor_local_dir =
                        crate::simulation::core::transforms::flu_vector_to_bevy_local_vector(
                            &nalgebra::Vector3::new(
                                ray.direction.x,
                                ray.direction.y,
                                ray.direction.z,
                            ),
                        );
                    // 2. Then, rotate it.
                    let world_direction: Vec3 = sensor_rotation * bevy_sensor_local_dir;

                    // Convert it to `Dir3` for the raycast.
                    if let Ok(dir) = Dir3::new(world_direction) {
                        let draw_length = if let Some(hit) = spatial_query.cast_ray(
                            sensor_origin,
                            dir,
                            max_toi,
                            true,
                            &filter,
                        ) {
                            hits.push(RayHit {
                                ray_id: ray.id,
                                distance: hit.distance,
                            });
                            hit.distance
                        } else {
                            max_toi
                        };
                        scan_cache.push((world_direction, draw_length));
                    }
                }
                sensor.last_scan = scan_cache;

                let agent_handle = FrameHandle::from_entity(agent_entity);
                let sensor_handle = FrameHandle::from_entity(sensor_entity);

                // Pass the raw hits back to the model to apply noise and create the final data packet.
                let measurement_data =
                    sensor
                        .model
                        .process_hits(&hits, sensor_handle, time.elapsed_secs_f64());

                // External path: for PointCloud data publish Arc to the bus so bridge
                // consumers share the allocation without copying the point Vec.
                if let MeasurementData::PointCloud(ref pc) = measurement_data {
                    topic_bus.publish_arc(&sensor.topic_name, Arc::new(pc.clone()));
                }

                let pure_message = MeasurementMessage {
                    agent_handle,
                    sensor_handle,
                    timestamp: time.elapsed_secs_f64(),
                    data: measurement_data,
                };

                // Internal path: Bevy event for the mapping system.
                measurement_writer.write(BevyMeasurementMessage(pure_message));
            }
        }
    }
}
