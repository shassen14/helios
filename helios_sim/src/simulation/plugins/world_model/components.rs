// helios_sim/src/plugins/world_model/components.rs

use bevy::{
    prelude::{Component, Entity},
    time::{Timer, TimerMode},
};
use helios_core::{estimation::StateEstimator, mapping::Mapper, slam::SlamSystem, types::Control};

/// The single, unified component that holds the autonomy logic for an agent.
#[derive(Component)]
pub enum WorldModelComponent {
    /// Configuration for separate localization and mapping modules.
    Separate {
        estimator: Box<dyn StateEstimator>,
        // For now, we'll use a placeholder `None` mapper.
        mapper: Box<dyn Mapper>,
    },
    /// Configuration for a unified SLAM system.
    CombinedSlam { system: Box<dyn SlamSystem> },
}

#[derive(Component, Default, Clone)]
pub struct ControlInputCache {
    /// The last known control input `u` for this agent.
    /// This value persists across frames until a new one arrives.
    pub u: Control,
}

/// Marker on a virtual "odom frame" entity that links it back to its agent.
///
/// The odom entity carries `TrackedFrame` + `Transform` + `GlobalTransform`.
/// Its `Transform` is updated every tick by `update_odom_frames` to match
/// whatever pose source is active on the agent's `WorldModelComponent`.
///
/// Design intent: any pose source (EKF, SLAM, VIO) that writes to
/// `WorldModelComponent` automatically drives this frame — callers only
/// need to implement the trait, not touch the TF plumbing.
#[derive(Component)]
pub struct OdomFrameOf(pub Entity);

#[derive(Component)]
pub struct ModuleTimer(pub Timer);

impl ModuleTimer {
    /// Creates a new timer from a frequency in Hz.
    pub fn from_hz(hz: f32) -> Self {
        let mut timer = Timer::from_seconds(1.0 / hz, TimerMode::Repeating);
        // We call tick once with a full duration to ensure it runs on the first frame.
        timer.tick(timer.duration());
        Self(timer)
    }
}
