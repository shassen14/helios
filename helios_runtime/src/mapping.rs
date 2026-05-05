//! Polymorphic mapping driver used by `AutonomyPipeline`.
//!
//! [`MapDriver`] abstracts over real mapper execution (`MappingCore`) and the
//! simulation mock [`StaticMapProvider`] (pre-built zero-filled map for isolation
//! profiles). Both implementations are `Send + Sync` for parallel Bevy scheduling.

use helios_core::{mapping::MapData, messages::MeasurementMessage};
use nalgebra::{DMatrix, Isometry3};

use crate::{pipeline::MappingCore, runtime::AgentRuntime};

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
    /// Forward a batch of sensor measurements to all active map layers.
    ///
    /// Called every `FixedUpdate` tick. The slice contains all measurements that
    /// arrived since the last tick, sorted by timestamp.
    fn process_messages(&mut self, msgs: &[MeasurementMessage], runtime: &dyn AgentRuntime);

    /// Notify map layers that the robot's odometry frame has moved.
    ///
    /// Used by local-frame occupancy grids to recenter around the current pose.
    /// Called on a separate timer (configurable Hz) rather than every tick.
    fn process_pose_update(&mut self, pose: Isometry3<f64>);

    /// Returns the current map for the given layer key, or `None` if no such layer exists.
    ///
    /// The key matches the TOML `map_layers` entry name (e.g. `"local"`, `"global"`).
    fn get_map(&self, key: &str) -> Option<&MapData>;
}

/// Static pre-built map for mock profiles.
/// Dimensions come from the fixture TOML; no algorithm runs at runtime.
#[derive(Clone)]
pub struct StaticMapProvider {
    map: MapData,
    layer: String,
}

impl StaticMapProvider {
    pub fn from_fixture(
        width_m: f64,
        height_m: f64,
        resolution_m: f64,
        origin: Isometry3<f64>,
        layer: impl Into<String>,
    ) -> Self {
        let cols = (width_m / resolution_m).ceil() as usize;
        let rows = (height_m / resolution_m).ceil() as usize;
        Self {
            map: MapData::OccupancyGrid2D {
                origin,
                resolution: resolution_m,
                data: DMatrix::zeros(rows, cols),
                version: 0,
            },
            layer: layer.into(),
        }
    }
}

impl MapDriver for StaticMapProvider {
    fn process_messages(&mut self, _: &[MeasurementMessage], _: &dyn AgentRuntime) {}
    fn process_pose_update(&mut self, _: Isometry3<f64>) {}
    fn get_map(&self, key: &str) -> Option<&MapData> {
        if key == self.layer {
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
    fn get_map(&self, key: &str) -> Option<&MapData> {
        self.get_map(key)
    }
}
