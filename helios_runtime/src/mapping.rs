// helios_runtime/src/mapping.rs
//
// MapDriver trait, StaticMapProvider, and impl MapDriver for MappingCore.

use helios_core::{mapping::MapData, messages::MeasurementMessage};
use nalgebra::{DMatrix, Isometry3};

use crate::{pipeline::MappingCore, runtime::AgentRuntime, stage::PipelineLevel};

/// Polymorphic mapping driver used by [`AutonomyPipeline`](crate::pipeline::AutonomyPipeline).
///
/// Two implementations exist:
/// - `MappingCore` — drives one or more real [`Mapper`](helios_core::mapping::Mapper)
///   implementations (e.g. occupancy grid).
/// - [`StaticMapProvider`] — returns a pre-built zero-filled map; used by `PlanningOnly`
///   and `ControlOnly` simulation profiles where mapping is not the system under test.
///
/// Both implementations are `Send + Sync` to support parallel Bevy system scheduling.
pub trait MapDriver: Send + Sync {
    /// Forward a batch of sensor measurements to all active mappers.
    ///
    /// Called every `FixedUpdate` tick. The slice contains all measurements that
    /// arrived since the last tick, sorted by timestamp.
    fn process_messages(&mut self, msgs: &[MeasurementMessage], runtime: &dyn AgentRuntime);

    /// Notify mappers that the robot's odometry frame has moved.
    ///
    /// Used by local-frame occupancy grids to recenter around the current pose.
    /// Called on a separate timer (configurable Hz) rather than every tick.
    fn process_pose_update(&mut self, pose: Isometry3<f64>);

    /// Returns the current map at the given pipeline level, if one exists.
    ///
    /// Returns `None` when no mapper is registered at `level`, or when the map
    /// has not yet been initialized (e.g. before the first pose update).
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
