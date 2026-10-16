//! [`EstimationFrames`]: the body an estimate tracks and the root it holds the
//! body's pose in.

use crate::prelude::AgentId;
use crate::spatial::FrameId;

/// The two frames an estimator's state relates: the body it tracks and the
/// root frame it holds that body's pose, velocity and attitude in.
///
/// Process and measurement models build their state blocks and read the state
/// through these rather than naming an agent's frames themselves, so two
/// estimators of one body can each hold its pose in a root of their own.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EstimationFrames {
    /// The frame the estimate tracks: the body's `base_link`.
    pub body: FrameId,
    /// The frame the estimate holds the body's pose in.
    pub root: FrameId,
}

impl EstimationFrames {
    /// `agent`'s `base_link`, held in `agent`'s `odom`.
    pub fn odom(agent: AgentId) -> Self {
        Self {
            body: FrameId::base_link(agent.clone()),
            root: FrameId::odom(agent),
        }
    }
}
