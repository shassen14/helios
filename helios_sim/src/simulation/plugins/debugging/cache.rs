use bevy::prelude::*;
use helios_core::messages::MeasurementData;
use nalgebra::Vector3;

use super::components::DebugSensorCache;
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::transforms::FluVector;

/// Reads PointCloud events and stores the latest world-space points per sensor.
pub fn cache_sensor_data(
    mut events: EventReader<BevyMeasurementMessage>,
    transform_query: Query<&GlobalTransform>,
    mut cache: ResMut<DebugSensorCache>,
) {
    for msg in events.read() {
        let sensor_entity = msg.0.sensor_handle.to_entity();
        let Ok(transform) = transform_query.get(sensor_entity) else {
            continue;
        };

        let world_pts: Option<Vec<Vec3>> = match &msg.0.data {
            MeasurementData::PointCloud2D(pc) => Some(
                pc.points
                    .iter()
                    .map(|p| {
                        // 2D LiDAR points live in the sensor's XY plane (z=0 in sensor frame).
                        let local = Vec3::from(FluVector(Vector3::new(p.x, p.y, 0.0)));
                        transform.transform_point(local)
                    })
                    .collect(),
            ),
            MeasurementData::PointCloud3D(pc) => Some(
                pc.points
                    .iter()
                    .map(|p| {
                        let local = Vec3::from(FluVector(Vector3::new(
                            p.position.x,
                            p.position.y,
                            p.position.z,
                        )));
                        transform.transform_point(local)
                    })
                    .collect(),
            ),
            _ => None,
        };

        if let Some(pts) = world_pts {
            cache.point_clouds.insert(sensor_entity, pts);
        }
    }
}
