// helios_runtime/src/pipeline/mapping_core.rs
//
// MappingCore struct + impl.

use helios_core::{
    estimation::FilterContext,
    mapping::MapData,
    messages::{MeasurementMessage, ModuleInput},
};
use nalgebra::Isometry3;

use crate::{
    runtime::{AgentRuntime, TfProviderAdapter},
    stage::{LeveledMapper, PipelineLevel},
};

/// Mapping stage: leveled mappers.
/// When `slam_active` is true, global-level mappers are skipped (SLAM owns that slot).
pub struct MappingCore {
    pub mappers: Vec<LeveledMapper>,
    pub slam_active: bool,
}

impl MappingCore {
    /// Feed new sensor messages to all mappers (cheap log-odds update).
    pub fn process_messages(&mut self, inputs: &[MeasurementMessage], runtime: &dyn AgentRuntime) {
        let adapter = TfProviderAdapter(runtime);
        let context = FilterContext { tf: Some(&adapter) };

        for leveled_mapper in &mut self.mappers {
            if self.slam_active && leveled_mapper.level == PipelineLevel::Global {
                continue;
            }
            for msg in inputs {
                leveled_mapper
                    .mapper
                    .process(&ModuleInput::Measurement { message: msg }, &context);
            }
        }
    }

    /// Push an odom pose update into all mappers so the grid can recenter.
    pub fn process_pose_update(&mut self, pose: Isometry3<f64>) {
        let context = FilterContext { tf: None };
        for leveled_mapper in &mut self.mappers {
            leveled_mapper
                .mapper
                .process(&ModuleInput::PoseUpdate { pose }, &context);
        }
    }

    /// Current map at the given level.
    pub fn get_map(&self, level: &PipelineLevel) -> Option<&MapData> {
        self.mappers
            .iter()
            .find(|lm| &lm.level == level)
            .map(|lm| lm.mapper.get_map())
    }
}
