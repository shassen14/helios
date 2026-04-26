use helios_core::frames::FrameAwareState;
use helios_core::planning::types::Path;
use helios_core::prelude::{PathFollower, PathFollowerResult};
use helios_core::types::TrajectoryPoint;

pub struct PathFollowingCore {
    pub path_follower: Box<dyn PathFollower>,
    current_path: Option<Path>,
}

impl PathFollowingCore {
    pub fn new(path_follower: Box<dyn PathFollower>) -> Self {
        Self {
            path_follower,
            current_path: None,
        }
    }

    pub fn set_path(&mut self, path: Path) {
        self.current_path = Some(path.clone());
        self.path_follower.set_path(path);
    }

    pub fn get_path(&self) -> Option<&Path> {
        self.current_path.as_ref()
    }

    pub fn get_lookahead_waypoint(&self) -> Option<&TrajectoryPoint> {
        self.path_follower.as_ref().get_lookahead_waypoint()
    }

    pub fn step(&mut self, state: &FrameAwareState, dt: f64) -> Option<TrajectoryPoint> {
        let result = self.path_follower.compute(state, dt);

        match result {
            PathFollowerResult::NoPath => None,
            PathFollowerResult::GoalReached => None,
            PathFollowerResult::Active(t) => Some(t),
            PathFollowerResult::Error(msg) => {
                log::warn!("[PathFollowingCore] error: {}", msg);
                None
            }
        }
    }
}
