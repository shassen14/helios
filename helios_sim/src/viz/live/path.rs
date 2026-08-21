//! Planned-path gizmo — a polyline through each agent's current plan.
//!
//! Unlike the pose gizmos, `Path` is a *plural* channel: a stack may declare
//! several, and none of their names may be hardcoded here (that is the agent
//! profile's to name). So instead of a canonical accessor this pulls every
//! declared `Path` output via [`declared_outputs`]. Nothing is drawn before the
//! planner's first output or when the stack has no planner; the iterator is
//! simply empty.
//!
//! Each waypoint is a [`Point<Enu>`] — a pure ENU world-frame position — so its
//! coordinates convert straight through the sole ENU→Bevy helper.
//!
//! [`Point<Enu>`]: helios_core::frames::quantities::Point

use crate::{
    core::transforms::EnuVector, prelude::AutonomyPipelineComponent,
    viz::live::discovery::declared_outputs,
};

use bevy::{color, prelude::*};
use helios_core::prelude::Path;

/// Path polyline color. Amber, chosen to sit clear of the red/green/blue axes
/// of the pose triads it is drawn among.
// TODO: pull the path color from viz config once that surface exists.
const PATH_COLOR: color::Color = color::Color::Srgba(color::Srgba {
    red: 1.0,
    green: 0.65,
    blue: 0.0,
    alpha: 1.0,
});

pub fn path_update_system(query: Query<&AutonomyPipelineComponent>, mut gizmos: Gizmos) {
    for pipeline in &query {
        for path in declared_outputs::<Path>(&pipeline.0) {
            let waypoints: Vec<Vec3> = path
                .value
                .waypoints
                .iter()
                .map(|wp| Vec3::from(EnuVector(*wp.raw())))
                .collect();

            gizmos.linestrip(waypoints, PATH_COLOR);
        }
    }
}
