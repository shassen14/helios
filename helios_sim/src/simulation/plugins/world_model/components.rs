// helios_sim/src/plugins/world_model/components.rs

use bevy::{
    prelude::Component,
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
