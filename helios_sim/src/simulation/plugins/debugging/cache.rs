use bevy::prelude::*;
use nalgebra::Vector3;

use super::components::DebugSensorCache;
use crate::simulation::core::transforms::FluVector;
use crate::simulation::plugins::autonomy::components::AutonomyPipelineComponent;

use helios_core::data::envelope::SensorReading;
use helios_core::data::sensor::PointCloud2D;
use helios_runtime::port::SensorChannel;

/// Reads the latest PointCloud2D readings from each agent's pipeline bus and
/// stores world-space points in the debug cache for visualization.
pub fn cache_sensor_data(
    pipeline_query: Query<&AutonomyPipelineComponent>,
    transform_query: Query<&GlobalTransform>,
    mut cache: ResMut<DebugSensorCache>,
) {
    for pipeline_comp in &pipeline_query {
        let Some(stamped) = pipeline_comp
            .0
            .bus()
            .read::<Vec<SensorReading<PointCloud2D>>>(
                SensorChannel::of::<Vec<SensorReading<PointCloud2D>>>().into(),
            )
        else {
            continue;
        };

        // Per-sensor dedup on batch timestamp: re-transform only when the
        // bus actually has a new batch. Without this we'd re-transform the
        // same FLU-local points by the sensor's *current* world transform
        // every Update frame, making the points slide with the moving
        // vehicle between lidar fires. The cache stores the world points
        // at the moment of the laser hit — they stay still in the world
        // while the vehicle drives through them.
        let batch_ts = stamped.timestamp.0;
        for reading in &stamped.value {
            let sensor_entity = reading.sensor_handle.to_entity();
            if cache
                .last_batch_ts
                .get(&sensor_entity)
                .is_some_and(|&prev| prev >= batch_ts)
            {
                continue;
            }
            let Ok(transform) = transform_query.get(sensor_entity) else {
                continue;
            };

            let world_pts: Vec<Vec3> = reading
                .data
                .points
                .iter()
                .map(|p| {
                    let local = Vec3::from(FluVector(Vector3::new(p.x, p.y, 0.0)));
                    transform.transform_point(local)
                })
                .collect();

            cache.point_clouds.insert(sensor_entity, world_pts);
            cache.last_batch_ts.insert(sensor_entity, batch_ts);
        }
    }
}
