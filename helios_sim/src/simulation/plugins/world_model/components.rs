// helios_sim/src/simulation/plugins/world_model/components.rs

use bevy::{
    prelude::{Component, Entity},
    time::{Timer, TimerMode},
};
use helios_runtime::pipeline::AutonomyPipeline;

/// The single, unified component that holds the autonomy pipeline for an agent.
/// Wraps an AutonomyPipeline — Bevy-free, hardware-portable core logic.
#[derive(Component)]
pub struct WorldModelComponent(pub AutonomyPipeline);

/// Marker on a virtual "odom frame" entity that links it back to its agent.
///
/// The odom entity carries `TrackedFrame` + `Transform` + `GlobalTransform`.
/// Its `Transform` is updated every tick by `update_odom_frames` to match
/// the pipeline's current pose estimate.
#[derive(Component)]
pub struct OdomFrameOf(pub Entity);

/// Rate-limit timer for the mapper pose update.
/// Fires at the mapper's configured Hz so the grid recenters periodically.
#[derive(Component)]
pub struct ModuleTimer(pub Timer);

impl ModuleTimer {
    pub fn from_hz(hz: f32) -> Self {
        let mut timer = Timer::from_seconds(1.0 / hz, TimerMode::Repeating);
        // Tick once immediately so it fires on the very first frame.
        timer.tick(timer.duration());
        Self(timer)
    }
}
