// helios_sim/src/simulation/plugins/debugging/gizmos/covariance.rs
//
// Covariance ellipsoid visualization stub.
// Will read from AutonomyPipelineComponent once the gizmo layer is updated.

use bevy::prelude::*;

use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;

/// No-op stub. Covariance gizmo deferred until pipeline state access is wired.
pub fn draw_covariance_ellipsoid(_config: Res<DebugVisualizationConfig>, _gizmos: Gizmos) {}
