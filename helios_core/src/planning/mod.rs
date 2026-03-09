// helios_core/src/planning/mod.rs
//
// Planner trait. No concrete implementations in Phase 2.

use crate::control::TrajectoryPoint;
use crate::estimation::FilterContext;
use crate::frames::FrameAwareState;
use crate::mapping::MapData;

/// A stateful planner that generates a trajectory point given the current state and map.
pub trait Planner: Send + Sync {
    fn plan(
        &mut self,
        state: &FrameAwareState,
        map: &MapData,
        goal: Option<&TrajectoryPoint>,
        context: &FilterContext,
    ) -> Option<TrajectoryPoint>;
}
