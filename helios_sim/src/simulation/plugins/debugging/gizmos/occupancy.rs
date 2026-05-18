// helios_sim/src/simulation/plugins/debugging/gizmos/occupancy.rs
//
// Occupancy grid visualization stub.
// Will read from AutonomyPipelineComponent once the gizmo layer is updated.

use bevy::prelude::*;

use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;

/// No-op stub. Occupancy grid gizmo deferred until pipeline map access is wired.
pub fn draw_occupancy_grid(_config: Res<DebugVisualizationConfig>, _gizmos: Gizmos) {}
