// helios_sim/src/simulation/plugins/debugging/gizmos/error.rs
//
// Estimation error line visualization stub.
// Will read from AutonomyPipelineComponent once the gizmo layer is updated.

use bevy::prelude::*;

use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;

/// No-op stub. Error line gizmo deferred until pipeline state access is wired.
pub fn draw_estimation_error_line(_config: Res<DebugVisualizationConfig>, _gizmos: Gizmos) {}
