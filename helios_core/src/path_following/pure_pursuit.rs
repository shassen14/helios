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
            return PathFollowerResult::GoalReached;
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
    }

    fn get_lookahead_waypoint(&self) -> Option<&Point<Enu>> {
        self.path
            .as_ref()
            .map(|p| &p.waypoints[self.lookahead_index])
    }

    fn reset(&mut self) {
        self.path = None;
        self.lookahead_index = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::types::Path;

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
