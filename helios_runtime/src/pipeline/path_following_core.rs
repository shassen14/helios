use helios_core::frames::FrameAwareState;
use helios_core::planning::types::Path;
use helios_core::prelude::{PathFollower, PathFollowerResult};
use helios_core::types::TrajectoryPoint;

// TODO: Option<Box> or keep Box<>?
pub struct PathFollowingCore {
    pub path_follower: Box<dyn PathFollower>,
}

impl PathFollowingCore {
    pub fn set_path(&mut self, path: Path) {
        self.path_follower.set_path(path);
    }

    pub fn step(&mut self, state: &FrameAwareState, dt: f64) -> Option<TrajectoryPoint> {
        let result = self.path_follower.compute(state, dt);

        // TODO: GoalReached change from None to richer input
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
