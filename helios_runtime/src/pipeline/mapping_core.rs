// helios_runtime/src/pipeline/mapping_core.rs
//
// MappingCore struct + impl.

use std::collections::HashMap;

use helios_core::{
    estimation::FilterContext,
    mapping::{MapData, Mapper},
    messages::{MeasurementMessage, ModuleInput},
};
use nalgebra::Isometry3;

use crate::runtime::{AgentRuntime, TfProviderAdapter};

/// Mapping stage: named map layers keyed by their TOML layer name.
///
/// Each entry corresponds to one `map_layers` key from `AutonomyStack`.
/// The key is also the channel name consumed by downstream planners (`MapData @ "<key>"`).
pub struct MappingCore {
    pub mappers: HashMap<String, Box<dyn Mapper>>,
}

impl MappingCore {
    /// Feed new sensor messages to all map layers.
    pub fn process_messages(&mut self, inputs: &[MeasurementMessage], runtime: &dyn AgentRuntime) {
        let adapter = TfProviderAdapter(runtime);
        let context = FilterContext { tf: Some(&adapter) };

        for mapper in self.mappers.values_mut() {
            for msg in inputs {
                mapper.process(&ModuleInput::Measurement { message: msg }, &context);
            }
        }
    }

    /// Push an odom pose update into all map layers so rolling-window grids can recenter.
    pub fn process_pose_update(&mut self, pose: Isometry3<f64>) {
        let context = FilterContext { tf: None };
        for mapper in self.mappers.values_mut() {
            mapper.process(&ModuleInput::PoseUpdate { pose }, &context);
        }
    }

    /// Current map for the given layer key, or `None` if no such layer is registered.
    pub fn get_map(&self, key: &str) -> Option<&MapData> {
        self.mappers.get(key).map(|m| m.get_map())
    }
}
