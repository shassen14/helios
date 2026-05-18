use bevy::prelude::*;
use nalgebra::Vector3;

use super::components::DebugSensorCache;
use crate::simulation::core::transforms::FluVector;
use crate::simulation::plugins::autonomy::components::AutonomyPipelineComponent;

use helios_core::data::sensor::{PointCloud2D, SensorReading};
use helios_runtime::port::ChannelKey;

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
                ChannelKey::of::<Vec<SensorReading<PointCloud2D>>>(),
            )
        else {
            continue;
        };

        for reading in &stamped.value {
            let sensor_entity = reading.sensor_handle.to_entity();
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
        }
    }
}
