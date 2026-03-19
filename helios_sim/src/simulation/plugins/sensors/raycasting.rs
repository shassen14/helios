// helios_sim/src/simulation/plugins/sensors/raycasting.rs
use avian3d::prelude::{SpatialQuery, SpatialQueryFilter}; // Import the correct types
use bevy::prelude::*;
use std::time::Duration;

// --- Simulation Crate Imports ---
use crate::prelude::*;
use crate::simulation::config::structs::{LidarConfig, SensorConfig};
use crate::simulation::core::{
    app_state::SimulationSet, components::SensorTopicName, events::BevyMeasurementMessage,
};

// --- Core Library Imports ---
use helios_core::{
    messages::MeasurementMessage,
    models::perception::{
        lidar_2d::Lidar2DModel, // We'll import our concrete models here
        RayHit,
        RaycastingSensorModel,
    },
    types::FrameHandle,
};
use crate::simulation::core::transforms::FluVector;

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
                        // Future implementation would go here.
                        unimplemented!("Lidar3D spawning not yet implemented.");
                    }
                };

                // --- 2. Spawn the Sensor Entity ---
                let mut sensor_entity_commands = commands.spawn_empty();
                let sensor_entity = sensor_entity_commands.id();

                let topic_name = format!("/{}/sensors/{}", agent_name.as_str(), sensor_name);
                sensor_entity_commands.insert((
                    Name::new(format!("{}/{}", agent_name.as_str(), sensor_name)),
                    RaycastingSensor {
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / lidar_config.get_rate()),
                            TimerMode::Repeating,
                        ),
                        model: core_model,
                        topic_name: topic_name.clone(),
                    },
                    // Topic name for the cold-path telemetry system.
                    SensorTopicName(topic_name),
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

/// Runs every frame to simulate raycasting sensors and emit `BevyMeasurementMessage` events.
/// PointCloud data is not published to TopicBus (see CLAUDE.md: visual data → MCAP, not Foxglove).
fn raycasting_sensor_system(
    mut measurement_writer: EventWriter<BevyMeasurementMessage>,
    time: Res<Time>,
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
                let sensor_origin = sensor_transform.translation();
                let sensor_rotation = sensor_transform.rotation();
                let max_toi = sensor.model.get_max_range();

                // We create a filter that excludes the sensor's own parent agent.
                let filter = SpatialQueryFilter::from_excluded_entities([agent_entity]);

                for ray in local_rays {
                    // 1. Convert ray direction from sensor FLU frame to Bevy local frame.
                    //    FLU +X (Forward) → Bevy -Z, FLU +Y (Left) → Bevy -X, FLU +Z (Up) → Bevy +Y
                    let bevy_sensor_local_dir = Vec3::from(
                        FluVector(nalgebra::Vector3::new(
                            ray.direction.x,
                            ray.direction.y,
                            ray.direction.z,
                        )),
                    );
                    // 2. Then, rotate it.
                    let world_direction: Vec3 = sensor_rotation * bevy_sensor_local_dir;

                    // Convert it to `Dir3` for the raycast.
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

                let agent_handle = FrameHandle::from_entity(agent_entity);
                let sensor_handle = FrameHandle::from_entity(sensor_entity);

                // Pass the raw hits back to the model to apply noise and create the final data packet.
                let measurement_data =
                    sensor
                        .model
                        .process_hits(&hits, sensor_handle, time.elapsed_secs_f64());

                let pure_message = MeasurementMessage {
                    agent_handle,
                    sensor_handle,
                    timestamp: time.elapsed_secs_f64(),
                    data: measurement_data,
                };

                // Emit event — routed to SensorMailbox by route_sensor_messages.
                measurement_writer.write(BevyMeasurementMessage(pure_message));
            }
        }
    }
}
