//! Stage wrappers and level tags for the `AutonomyPipeline`.
//!
//! [`PipelineLevel`] tags each planner and controller so `PipelineBuilder`
//! can sort stages at build time (`Global` → `Local` → `Custom`). Map layers are
//! keyed by string name (matching TOML `map_layers` keys) rather than `PipelineLevel`.

use helios_core::{control::Controller, planning::GeometricPlanner};

/// Execution level for planners and controllers.
///
/// Stages are sorted by this at `PipelineBuilder::build()` time so `Global` always
/// runs before `Local`, which always runs before any `Custom` variant (lexicographic).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PipelineLevel {
    /// Global (world-scale) scope — e.g., a global planner.
    Global,
    /// Local (robot-centric) scope — e.g., a local reactive controller.
    Local,
    /// User-defined named scope for multi-layer architectures.
    Custom(String),
}

/// A planner with an associated level tag.
pub struct LeveledPlanner {
    pub level: PipelineLevel,
    pub planner: Box<dyn GeometricPlanner>,
}

/// A controller with an associated level tag.
pub struct LeveledController {
    pub level: PipelineLevel,
    pub controller: Box<dyn Controller>,
}
