use nalgebra::Isometry3;

use crate::data::sensor::PointCloud2D;
use crate::mapping::{MapData, Mapper};

/// A placeholder mapper that does nothing.
/// It is used when an agent is configured with an estimator but no mapping capability.
#[derive(Default, Debug, Clone)]
pub struct NoneMapper;

impl Mapper for NoneMapper {
    fn recenter(&mut self, _robot_world_pose: &Isometry3<f64>) {}

    fn integrate_scan_2d(
        &mut self,
        _sensor_world_pose: &Isometry3<f64>,
        _cloud: &PointCloud2D,
    ) {
    }

    fn get_map(&mut self) -> &MapData {
        // We can't return a temporary, so we create a static one.
        static EMPTY_MAP: MapData = MapData::None;
        &EMPTY_MAP
    }
}
