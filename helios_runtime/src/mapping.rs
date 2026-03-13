// helios_runtime/src/mapping.rs
//
// MapDriver trait, StaticMapProvider, and impl MapDriver for MappingCore.

use helios_core::{mapping::MapData, messages::MeasurementMessage};
use nalgebra::{DMatrix, Isometry3};

use crate::{pipeline::MappingCore, runtime::AgentRuntime, stage::PipelineLevel};

/// Polymorphic mapping driver. Implemented by both `MappingCore` (real mapper)
/// and `StaticMapProvider` (sim-only mock with fixed dimensions from a fixture).
pub trait MapDriver: Send + Sync {
    fn process_messages(&mut self, msgs: &[MeasurementMessage], runtime: &dyn AgentRuntime);
    fn process_pose_update(&mut self, pose: Isometry3<f64>);
    fn get_map(&self, level: &PipelineLevel) -> Option<&MapData>;
}

/// Static pre-built map for mock profiles.
/// Dimensions come from the fixture TOML; no algorithm runs at runtime.
#[derive(Clone)]
pub struct StaticMapProvider {
    map: MapData,
    level: PipelineLevel,
}

impl StaticMapProvider {
    pub fn from_fixture(
        width_m: f64,
        height_m: f64,
        resolution_m: f64,
        origin: Isometry3<f64>,
        level: PipelineLevel,
    ) -> Self {
        let cols = (width_m / resolution_m).ceil() as usize;
        let rows = (height_m / resolution_m).ceil() as usize;
        Self {
            map: MapData::OccupancyGrid2D {
                origin,
                resolution: resolution_m,
                data: DMatrix::zeros(rows, cols),
            },
            level,
        }
    }
}

impl MapDriver for StaticMapProvider {
    fn process_messages(&mut self, _: &[MeasurementMessage], _: &dyn AgentRuntime) {}
    fn process_pose_update(&mut self, _: Isometry3<f64>) {}
    fn get_map(&self, level: &PipelineLevel) -> Option<&MapData> {
        if *level == self.level {
            Some(&self.map)
        } else {
            None
        }
    }
}

impl MapDriver for MappingCore {
    fn process_messages(&mut self, msgs: &[MeasurementMessage], runtime: &dyn AgentRuntime) {
        self.process_messages(msgs, runtime);
    }
    fn process_pose_update(&mut self, pose: Isometry3<f64>) {
        self.process_pose_update(pose);
    }
    fn get_map(&self, level: &PipelineLevel) -> Option<&MapData> {
        self.get_map(level)
    }
}
