use super::{PathFollower, PathFollowerInputs, PathFollowerResult};
use crate::control::commands::BodyTwist;
use crate::control::BodyTwistRef;
use crate::data::primitives::FrameHandle;
use crate::frames::conventions::{Enu, Flu};
use crate::frames::quantities::Point;
use crate::frames::{FrameAwareState, FrameId};
use crate::planning::types::Path;
use nalgebra::Vector2;
// Plan: output (velocity.x, angle.z)
//          dot (accel.x, ang_vel.z) -> curvature, but fails

pub struct PurePursuitPathFollower {
    lookahead_distance: f64,
    lookahead_time: Option<f64>,
    goal_radius: f64,
    min_speed: f64,
    max_speed: f64,
    max_lateral_acceleration: f64,
    path: Option<Path>,
    lookahead_index: usize,
    /// Latches once the agent first reaches `goal_radius` of the final waypoint.
    /// While set, `compute` returns the stop reference regardless of the agent's
    /// current position — an inertial body coasting back outside the radius does
    /// not re-arm driving. Cleared only by `set_path`/`reset`, so a completed
    /// path stays completed until the planner issues a new one.
    arrived: bool,
    agent_handle: FrameHandle,
}

impl PurePursuitPathFollower {
    pub fn new(
        lookahead_distance: f64,
        lookahead_time: Option<f64>,
        goal_radius: f64,
        min_speed: f64,
        max_speed: f64,
        max_lateral_acceleration: f64,
        agent_handle: FrameHandle,
    ) -> Self {
        Self {
            lookahead_distance,
            lookahead_time,
            goal_radius,
            min_speed,
            max_speed,
            max_lateral_acceleration,
            path: None,
            lookahead_index: 0,
            arrived: false,
            agent_handle,
        }
    }

    fn calculate_lookahead_index(&mut self, state: &FrameAwareState, lookahead_distance: f64) {
        let Some(path) = &self.path else {
            return;
        };

        let agent_pos = match state.position::<Enu>(FrameId::Odom(self.agent_handle)) {
            Some(p) => Vector2::new(p.x(), p.y()),
            None => return,
        };

        // Linearly go from the last starting point of the path
        // forward to determine the index to look at that is
        // at least lookahead_distance away or the end of the path
        while self.lookahead_index + 1 < path.waypoints.len() {
            let path_pos = Vector2::new(
                path.waypoints[self.lookahead_index].x(),
                path.waypoints[self.lookahead_index].y(),
            );

            let distance = (agent_pos - path_pos).norm();

            if distance < lookahead_distance {
                self.lookahead_index += 1;
            } else {
                break;
            }
        }
    }
}

impl PathFollower for PurePursuitPathFollower {
    type Reference = BodyTwistRef;
    fn compute(
        &mut self,
        _dt: f64,
        inputs: &PathFollowerInputs,
    ) -> PathFollowerResult<BodyTwistRef> {
        // A completed path stays completed: once arrived, hold the stop reference
        // no matter where the body has since drifted, so an inertial vehicle that
        // coasts past the goal does not re-arm and drive back.
        if self.arrived {
            return PathFollowerResult::GoalReached(BodyTwistRef::new(BodyTwist::zero()));
        }

        let state = &inputs.state;

        let lookahead_distance: f64 = match self.lookahead_time {
            Some(t) => {
                let speed = state
                    .velocity::<Enu>(FrameId::Odom(self.agent_handle))
                    .map(|v| v.raw().xy().norm())
                    .unwrap_or(0.0);
                (speed * t).max(self.lookahead_distance)
            }
            None => self.lookahead_distance,
        };

        self.calculate_lookahead_index(state, lookahead_distance);

        let Some(path) = &self.path else {
            return PathFollowerResult::NoPath;
        };

        let agent_pos = match state.position::<Enu>(FrameId::Odom(self.agent_handle)) {
            Some(p) => Vector2::new(p.x(), p.y()),
            None => {
                return PathFollowerResult::Error(
                    "Could not abstract agent information".to_string(),
                )
            }
        };
        let agent_orientation = match state.orientation::<Flu, Enu>(
            FrameId::Body(self.agent_handle),
            FrameId::Odom(self.agent_handle),
        ) {
            Some(o) => o.into_inner(),
            None => {
                return PathFollowerResult::Error(
                    "Could not abstract agent information".to_string(),
                )
            }
        };

        let (_, _, agent_yaw) = agent_orientation.euler_angles();

        let lookahead_pos = Vector2::new(
            path.waypoints[self.lookahead_index].x(),
            path.waypoints[self.lookahead_index].y(),
        );

        let delta = lookahead_pos - agent_pos;
        let distance = delta.norm();

        if distance <= self.goal_radius {
            self.arrived = true;
            return PathFollowerResult::GoalReached(BodyTwistRef::new(BodyTwist::zero()));
        }

        let lookahead_yaw = delta.y.atan2(delta.x);
        let bearing_to_target = lookahead_yaw - agent_yaw;
        let curvature = 2.0 * f64::sin(bearing_to_target) / lookahead_distance;

        let forward_velocity_desired = f64::sqrt(self.max_lateral_acceleration / curvature.abs())
            .clamp(self.min_speed, self.max_speed);

        let angular_velocity_desired = forward_velocity_desired * curvature;

        let body_twist = BodyTwist::unicycle(forward_velocity_desired, angular_velocity_desired);

        let reference = BodyTwistRef::new(body_twist);

        PathFollowerResult::Active(reference)
    }

    fn set_path(&mut self, path: Path) {
        self.path = Some(path);
        self.lookahead_index = 0;
        self.arrived = false;
    }

    fn get_lookahead_waypoint(&self) -> Option<&Point<Enu>> {
        self.path
            .as_ref()
            .map(|p| &p.waypoints[self.lookahead_index])
    }

    fn reset(&mut self) {
        self.path = None;
        self.lookahead_index = 0;
        self.arrived = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimation::schema::{StateSchemaBlock, StateSchema};
    use crate::frames::transforms::Convention;
    use crate::manifold::TangentNoise;
    use crate::planning::types::Path;
    use crate::state::Quantity;

    use nalgebra::{DMatrix, DVector};
    use std::sync::Arc;

    fn follower() -> PurePursuitPathFollower {
        // Only the path plumbing is exercised here; the tuning values are
        // arbitrary and the lookahead accessor ignores the agent handle.
        PurePursuitPathFollower::new(1.0, None, 0.5, 0.1, 2.0, 1.0, FrameHandle(0))
    }

    fn path_of(waypoints: Vec<Point<Enu>>) -> Path {
        Path {
            waypoints,
            timestamp: 0.0,
            level_key: "global".into(),
        }
    }

    // Orientation blocks refuse `None` noise, so seed every block with the same
    // isotropic 3-DOF variance; its value is irrelevant to a geometry test.
    fn noise3() -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap())
    }

    /// A state carrying `handle`'s Odom position `(x, y, 0)` and an identity
    /// Body→Odom orientation — the two blocks `compute` reads to locate the agent.
    fn inputs_at(handle: FrameHandle, x: f64, y: f64) -> PathFollowerInputs {
        let schema = StateSchema::compose(vec![
            StateSchemaBlock::new(
                Quantity::Position(FrameId::Odom(handle)),
                Convention::Enu,
                noise3(),
                DVector::from_vec(vec![x, y, 0.0]),
                DMatrix::identity(3, 3),
            ),
            StateSchemaBlock::orientation(
                FrameId::Body(handle),
                FrameId::Odom(handle),
                Convention::Flu,
                Convention::Enu,
                noise3(),
                DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
                DMatrix::identity(3, 3),
            ),
        ]);
        PathFollowerInputs {
            state: FrameAwareState::from_schema(Arc::new(schema), 0.0),
        }
    }

    /// Arrival latches: once the agent first reaches `goal_radius` of the final
    /// waypoint, the follower keeps returning `GoalReached` (the stop reference)
    /// even after the body drifts well outside the radius. Without the latch an
    /// inertial vehicle that coasts past the goal re-arms `Active` and drives
    /// back, oscillating — the drift call below would return `Active`.
    #[test]
    fn arrival_latches_and_does_not_rearm_on_drift() {
        let handle = FrameHandle(7);
        // goal_radius = 1.0; a single-waypoint path at the origin.
        let mut f = PurePursuitPathFollower::new(1.0, None, 1.0, 0.1, 2.0, 1.0, handle);
        f.set_path(path_of(vec![Point::<Enu>::new(0.0, 0.0, 0.0)]));

        // Inside the radius (0.5 m) → arrives and returns a stop.
        let arrived = f.compute(0.1, &inputs_at(handle, 0.5, 0.0));
        assert!(matches!(arrived, PathFollowerResult::GoalReached(_)));

        // Drifted 5 m away — well outside the radius. Latched → still GoalReached,
        // and the carried reference is the zero-twist stop, not a drive-back.
        match f.compute(0.1, &inputs_at(handle, 5.0, 0.0)) {
            PathFollowerResult::GoalReached(r) => assert_eq!(*r.twist(), BodyTwist::zero()),
            _ => panic!("latched follower re-armed after drifting outside goal_radius"),
        }
    }

    /// After `set_path`, the lookahead cursor starts at the first waypoint and the
    /// typed `Point<Enu>` handed back carries the exact planned coordinates: the
    /// position round-trips through the follower's storage unchanged. This is the
    /// regression guard for the waypoint carrier being a typed point rather than a
    /// layout-indexed state vector.
    #[test]
    fn lookahead_waypoint_round_trips_planned_position() {
        let waypoints = vec![
            Point::<Enu>::new(1.0, 2.0, 0.0),
            Point::<Enu>::new(3.0, 4.0, 0.0),
        ];
        let mut f = follower();
        f.set_path(path_of(waypoints.clone()));

        let wp = f.get_lookahead_waypoint().expect("a path was set");
        assert_eq!(*wp, waypoints[0]);
        assert_eq!((wp.x(), wp.y(), wp.z()), (1.0, 2.0, 0.0));
    }

    /// With no path set, there is no lookahead waypoint to report.
    #[test]
    fn no_lookahead_waypoint_before_set_path() {
        assert!(follower().get_lookahead_waypoint().is_none());
    }
}
