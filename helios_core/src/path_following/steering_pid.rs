// helios_core/src/path_following/steering_pid.rs
//
// SteeringPidPathFollower: heading-error path follower implemented as a PathFollower.
// Same PID logic as the old SteeringPidController, but advances its own lookahead
// index along the path and emits body-twist references for the controller stage.
//
// Use this as a diagnostic: if this produces correct steering through the
// PathFollowing → DirectTwist → DualSisoPid chain, the wiring is correct
// and any issues with PurePursuit are in that algorithm's geometry.

use super::{PathFollower, PathFollowerInputs, PathFollowerResult};
use crate::control::commands::BodyTwist;
use crate::control::kernels::siso_pid::SisoPid;
use crate::control::BodyTwistRef;
use crate::data::primitives::FrameHandle;
use crate::frames::conventions::{Enu, Flu};
use crate::frames::quantities::Point;
use crate::frames::FrameId;
use crate::planning::types::Path;

use nalgebra::Vector2;

pub struct SteeringPidPathFollower {
    heading_pid: SisoPid,
    cruise_speed: f64,
    goal_radius: f64,
    lookahead_distance: f64,
    path: Option<Path>,
    lookahead_index: usize,
    /// Latches once the agent first reaches `goal_radius` of the final waypoint.
    /// While set, `compute` returns the stop reference regardless of the agent's
    /// current position, so an inertial body coasting back outside the radius
    /// does not re-arm driving. Cleared only by `set_path`/`reset`.
    arrived: bool,
    agent_handle: FrameHandle,
}

impl SteeringPidPathFollower {
    pub fn new(
        kp: f64,
        ki: f64,
        kd: f64,
        cruise_speed: f64,
        goal_radius: f64,
        lookahead_distance: f64,
        agent_handle: FrameHandle,
    ) -> Self {
        Self {
            heading_pid: SisoPid::new(kp, ki, kd),
            cruise_speed,
            goal_radius,
            lookahead_distance,
            path: None,
            lookahead_index: 0,
            arrived: false,
            agent_handle,
        }
    }

    fn advance_lookahead(&mut self, agent_pos: Vector2<f64>) {
        let Some(path) = &self.path else { return };
        while self.lookahead_index + 1 < path.waypoints.len() {
            let wp = path.waypoints[self.lookahead_index];
            let path_pos = Vector2::new(wp.x(), wp.y());
            if (agent_pos - path_pos).norm() < self.lookahead_distance {
                self.lookahead_index += 1;
            } else {
                break;
            }
        }
    }
}

fn normalize_angle(a: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let a = ((a % two_pi) + two_pi) % two_pi;
    if a > std::f64::consts::PI {
        a - two_pi
    } else {
        a
    }
}

impl PathFollower for SteeringPidPathFollower {
    type Reference = BodyTwistRef;

    fn compute(
        &mut self,
        dt: f64,
        inputs: &PathFollowerInputs,
    ) -> PathFollowerResult<BodyTwistRef> {
        // A completed path stays completed: once arrived, hold the stop reference
        // regardless of where the body has since drifted.
        if self.arrived {
            return PathFollowerResult::GoalReached(BodyTwistRef::new(BodyTwist::zero()));
        }

        let state = &inputs.state;

        if self.path.is_none() {
            return PathFollowerResult::NoPath;
        }

        let agent_pos = match state.position::<Enu>(FrameId::Odom(self.agent_handle)) {
            Some(p) => Vector2::new(p.x(), p.y()),
            None => return PathFollowerResult::Error("missing agent position".into()),
        };
        let orientation = match state.orientation::<Flu, Enu>(
            FrameId::Body(self.agent_handle),
            FrameId::Odom(self.agent_handle),
        ) {
            Some(q) => q.into_inner(),
            None => return PathFollowerResult::Error("missing agent orientation".into()),
        };
        let (_, _, current_yaw) = orientation.euler_angles();

        self.advance_lookahead(agent_pos);

        let lookahead_pos = match self
            .path
            .as_ref()
            .and_then(|p| p.waypoints.get(self.lookahead_index))
        {
            Some(wp) => Vector2::new(wp.x(), wp.y()),
            None => return PathFollowerResult::Error("missing waypoint position".into()),
        };

        let dist = (lookahead_pos - agent_pos).norm();

        if dist < self.goal_radius {
            self.arrived = true;
            self.heading_pid.reset();
            return PathFollowerResult::GoalReached(BodyTwistRef::new(BodyTwist::zero()));
        }

        let delta = lookahead_pos - agent_pos;

        let desired_yaw = delta.y.atan2(delta.x);
        let heading_error = normalize_angle(desired_yaw - current_yaw);
        let wz = self.heading_pid.update(heading_error, dt);

        let body_twist = BodyTwist::unicycle(self.cruise_speed, wz);

        let reference = BodyTwistRef::new(body_twist);

        PathFollowerResult::Active(reference)
    }

    fn set_path(&mut self, path: Path) {
        self.path = Some(path);
        self.lookahead_index = 0;
        self.arrived = false;
        self.heading_pid.reset();
    }

    fn get_lookahead_waypoint(&self) -> Option<&Point<Enu>> {
        self.path
            .as_ref()
            .and_then(|p| p.waypoints.get(self.lookahead_index))
    }

    fn reset(&mut self) {
        self.path = None;
        self.lookahead_index = 0;
        self.arrived = false;
        self.heading_pid.reset();
    }
}
