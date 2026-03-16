//! Stage wrappers and level tags for the `AutonomyPipeline`.
//!
//! [`PipelineLevel`] tags each mapper, planner, and controller so `PipelineBuilder`
//! can sort stages at build time (`Global` → `Local` → `Custom`). The three leveled
//! wrappers (`LeveledMapper`, `LeveledPlanner`, `LeveledController`) pair an algorithm
//! with its assigned level for the pipeline's stage vectors.

use helios_core::{control::Controller, mapping::Mapper, planning::Planner};

/// Execution level for mappers, planners, and controllers.
///
/// Stages are sorted by this at `PipelineBuilder::build()` time so `Global` always
/// runs before `Local`, which always runs before any `Custom` variant (lexicographic).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PipelineLevel {
    /// Global (world-scale) scope — e.g., a global occupancy grid or global planner.
    Global,
    /// Local (robot-centric) scope — e.g., a local costmap or local reactive controller.
    Local,
    /// User-defined named scope for multi-layer architectures.
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
