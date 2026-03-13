// helios_runtime/src/stage.rs
//
// Stage kinds and level tags for the AutonomyPipeline.
// Stages are sorted by PipelineLevel at build time: Global → Local → Custom.

use helios_core::{control::Controller, mapping::Mapper, planning::Planner};

/// Execution level for mappers, planners, and controllers.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PipelineLevel {
    Global,
    Local,
    Custom(String),
}

/// A mapper with an associated level tag.
pub struct LeveledMapper {
    pub level: PipelineLevel,
    pub mapper: Box<dyn Mapper>,
}

/// A planner with an associated level tag.
pub struct LeveledPlanner {
    pub level: PipelineLevel,
    pub planner: Box<dyn Planner>,
}

/// A controller with an associated level tag.
pub struct LeveledController {
    pub level: PipelineLevel,
    pub controller: Box<dyn Controller>,
}
