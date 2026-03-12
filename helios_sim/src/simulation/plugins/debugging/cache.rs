use bevy::prelude::*;
use helios_core::messages::MeasurementData;

use super::components::DebugSensorCache;
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::transforms::flu_vector_to_bevy_local_vector;

/// Reads PointCloud events and stores the latest world-space points per sensor.
pub fn cache_sensor_data(
    mut events: EventReader<BevyMeasurementMessage>,
    transform_query: Query<&GlobalTransform>,
    mut cache: ResMut<DebugSensorCache>,
) {
    for msg in events.read() {
        if let MeasurementData::PointCloud(ref pc) = msg.0.data {
            let sensor_entity = msg.0.sensor_handle.to_entity();
            if let Ok(transform) = transform_query.get(sensor_entity) {
                let world_pts: Vec<Vec3> = pc
                    .points
                    .iter()
                    .map(|p| {
                        let flu = nalgebra::Vector3::new(p.position.x, p.position.y, p.position.z);
                        let local = flu_vector_to_bevy_local_vector(&flu);
                        transform.transform_point(local)
                    })
                    .collect();
                cache.point_clouds.insert(sensor_entity, world_pts);
            }
        }
    }
}
