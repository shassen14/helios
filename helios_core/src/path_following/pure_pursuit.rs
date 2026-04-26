use super::{PathFollower, PathFollowerResult};
use crate::prelude::FrameAwareState;
use crate::prelude::FrameId;
use crate::prelude::Path;
use crate::prelude::StateVariable;
use crate::types::FrameHandle;
use crate::types::TrajectoryPoint;
use nalgebra::Vector2;
// Plan: output (velocity.x, angle.z)
//          dot (accel.x, ang_vel.z) -> curvature, but fails
pub struct PurePursuitPathFollower {
    wheelbase: f64,
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
        wheelbase: f64,
        lookahead_distance: f64,
        lookahead_time: Option<f64>,
        goal_radius: f64,
        min_speed: f64,
        max_speed: f64,
        max_lateral_acceleration: f64,
        agent_handle: FrameHandle,
    ) -> Self {
        Self {
            wheelbase,
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

        let agent_pos = match state.get_vector3(&StateVariable::Px(FrameId::World)) {
            Some(p) => Vector2::new(p.x, p.y),
            None => return,
        };

        // Linearly go from the last starting point of the path
        // forward to determine the index to look at that is
        // at least lookahead_distance away or the end of the path
        while self.lookahead_index + 1 < path.waypoints.len() {
            let path_pos = match &path.waypoints[self.lookahead_index]
                .state
                .get_vector3(&StateVariable::Px(FrameId::World))
            {
                Some(p) => Vector2::new(p.x, p.y),
                None => return,
            };

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
    fn compute(&mut self, state: &FrameAwareState, _dt: f64) -> PathFollowerResult {
        // we have current state
        // output desired state being v_x, yaw
        // if lookahead_time_s exists -> calc the distance
        //   with current velocity
        // else -> distance = lookahead_distance
        //
        // if Path doesn't exist -> NoPath
        //
        // find the lookahead_index comparing distances
        //
        // using this we can probably calculate
        // velocity and angular velocity
        // w = curvature * velocity
        //

        let mut lookahead_distance: f64 = self.lookahead_distance;

        if self.lookahead_time.is_some() {
            // lookahead_distance = ;
        }

        self.calculate_lookahead_index(state, lookahead_distance);

        let Some(path) = &self.path else {
            return PathFollowerResult::NoPath;
        };

        let agent_pos = match state.get_vector3(&StateVariable::Px(FrameId::World)) {
            Some(p) => Vector2::new(p.x, p.y),
            None => {
                return PathFollowerResult::Error(
                    "Could not abstract agent information".to_string(),
                )
            }
        };
        let agent_orientation = match state.get_orientation() {
            Some(o) => o,
            None => {
                return PathFollowerResult::Error(
                    "Could not abstract agent information".to_string(),
                )
            }
        };

        let (_, _, agent_yaw) = agent_orientation.euler_angles();

        let lookahead_pos = match path.waypoints[self.lookahead_index]
            .state
            .get_vector3(&StateVariable::Px(FrameId::World))
        {
            Some(p) => Vector2::new(p.x, p.y),
            None => {
                return PathFollowerResult::Error(
                    "Could not abstract waypoint information".to_string(),
                )
            }
        };

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

        let body_handle = FrameId::Body(self.agent_handle);
        let layout = vec![
            StateVariable::Vx(body_handle.clone()),
            StateVariable::Wz(body_handle.clone()),
        ];

        let mut state_desired = FrameAwareState::new(layout, 0.0f64, state.last_update_timestamp);
        state_desired.vector[0] = forward_velocity_desired;
        state_desired.vector[1] = angular_velocity_desired;

        let point_desired = TrajectoryPoint {
            state: state_desired,
            state_dot: None,
            time: state.last_update_timestamp,
        };

        PathFollowerResult::Active(point_desired)
    }

    fn set_path(&mut self, path: Path) {
        self.path = Some(path);
        self.lookahead_index = 0;
    }

    fn reset(&mut self) {
        self.path = None;
        self.lookahead_index = 0;
    }
}
