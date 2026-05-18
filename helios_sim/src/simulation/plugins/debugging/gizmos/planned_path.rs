// helios_sim/src/simulation/plugins/debugging/gizmos/planned_path.rs
//
// Planned path visualization stub.
// Will read from AutonomyPipelineComponent once path following is wired.

use bevy::prelude::*;

use super::super::DebugVisualizationConfig;

/// No-op stub. Planned path gizmo deferred until path following is wired to the pipeline.
pub fn draw_planned_path(_config: Res<DebugVisualizationConfig>, _gizmos: Gizmos) {}
