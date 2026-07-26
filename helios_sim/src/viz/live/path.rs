//! Planned-path gizmo — a polyline through each agent's current plan.
//!
//! Unlike the pose gizmos, `Path` is a *plural* channel: a stack may declare
//! several, and none of their names may be hardcoded here (that is the agent
//! profile's to name). So instead of a canonical accessor this pulls every
//! declared `Path` output via [`declared_outputs`]. Nothing is drawn before the
//! planner's first output or when the stack has no planner; the iterator is
//! simply empty.
//!
//! Each waypoint is a full [`RobotState`], not a bare point (the real consumer,
//! the path follower, needs heading and feed-forward), so the position is
//! pulled out per waypoint and converted through the sole ENU→Bevy helper.
//!
//! [`RobotState`]: helios_core::frames::RobotState

use crate::{
    core::transforms::EnuVector,
    prelude::AutonomyPipelineComponent,
    viz::live::discovery::declared_outputs,
};

use bevy::{color, prelude::*};
use helios_core::{
    frames::{FrameId, StateVariable},
    prelude::Path,
};

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
                .filter_map(|wp| wp.state.get_vector3(&StateVariable::Px(FrameId::World)))
                .map(|v| Vec3::from(EnuVector(v)))
                .collect();

            gizmos.linestrip(waypoints, PATH_COLOR);
        }
    }
}
