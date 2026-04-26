// helios_core/src/path_following/steering_pid.rs
//
// SteeringPidPathFollower: heading-error path follower implemented as a PathFollower.
// Same PID logic as the old SteeringPidController, but advances its own lookahead
// index along the path and emits TrajectoryPoints for the controller stage.
//
// Use this as a diagnostic: if this produces correct steering through the
// PathFollowing → DirectVelocity → DualSisoPid chain, the wiring is correct
// and any issues with PurePursuit are in that algorithm's geometry.

use nalgebra::Vector2;

use super::{PathFollower, PathFollowerResult};
use crate::control::siso_pid::SisoPid;
use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::planning::types::Path;
use crate::types::{FrameHandle, TrajectoryPoint};

pub struct SteeringPidPathFollower {
    heading_pid: SisoPid,
    cruise_speed: f64,
    goal_radius: f64,
    lookahead_distance: f64,
    path: Option<Path>,
    lookahead_index: usize,
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
            agent_handle,
        }
    }

    fn advance_lookahead(&mut self, agent_pos: Vector2<f64>) {
        let Some(path) = &self.path else { return };
        while self.lookahead_index + 1 < path.waypoints.len() {
            let path_pos = match path.waypoints[self.lookahead_index]
                .state
                .get_vector3(&StateVariable::Px(FrameId::World))
            {
                Some(p) => Vector2::new(p.x, p.y),
                None => return,
            };
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
    if a > std::f64::consts::PI { a - two_pi } else { a }
}

impl PathFollower for SteeringPidPathFollower {
    fn compute(&mut self, state: &FrameAwareState, dt: f64) -> PathFollowerResult {
        if self.path.is_none() {
            return PathFollowerResult::NoPath;
        }

        let agent_pos = match state.get_vector3(&StateVariable::Px(FrameId::World)) {
            Some(p) => Vector2::new(p.x, p.y),
            None => return PathFollowerResult::Error("missing agent position".into()),
        };
        let orientation = match state.get_orientation() {
            Some(q) => q,
            None => return PathFollowerResult::Error("missing agent orientation".into()),
        };
        let (_, _, current_yaw) = orientation.euler_angles();

        self.advance_lookahead(agent_pos);

        let lookahead_pos = match self.path.as_ref().unwrap().waypoints[self.lookahead_index]
            .state
            .get_vector3(&StateVariable::Px(FrameId::World))
        {
            Some(p) => Vector2::new(p.x, p.y),
            None => return PathFollowerResult::Error("missing waypoint position".into()),
        };

        let dist = (lookahead_pos - agent_pos).norm();

        if dist < self.goal_radius {
            self.heading_pid.reset();
            return PathFollowerResult::GoalReached;
        }

        let delta = lookahead_pos - agent_pos;

        let desired_yaw = delta.y.atan2(delta.x);
        let heading_error = normalize_angle(desired_yaw - current_yaw);
        let wz = self.heading_pid.update(heading_error, dt);

        let body_id = FrameId::Body(self.agent_handle);
        let layout = vec![
            StateVariable::Vx(body_id.clone()),
            StateVariable::Wz(body_id.clone()),
        ];
        let mut ref_state = FrameAwareState::new(layout, 0.0, state.last_update_timestamp);
        ref_state.vector[0] = self.cruise_speed;
        ref_state.vector[1] = wz;

        PathFollowerResult::Active(TrajectoryPoint {
            state: ref_state,
            state_dot: None,
            time: state.last_update_timestamp,
        })
    }

    fn set_path(&mut self, path: Path) {
        self.path = Some(path);
        self.lookahead_index = 0;
        self.heading_pid.reset();
    }

    fn reset(&mut self) {
        self.path = None;
        self.lookahead_index = 0;
        self.heading_pid.reset();
    }
}
