// helios_sim/src/simulation/plugins/autonomy/components.rs

use bevy::{
    prelude::{Component, Entity},
    time::{Timer, TimerMode},
};
use helios_runtime::pipeline::{ControlCore, EstimationCore, MappingCore};

/// Holds the estimation stage for an agent: estimator/SLAM + trackers + predict/update logic.
/// Systems that need estimated state query this component.
#[derive(Component)]
pub struct EstimatorComponent(pub EstimationCore);

/// Holds the mapping stage for an agent: leveled mappers.
/// Systems that need map data query this component.
#[derive(Component)]
pub struct MapperComponent(pub MappingCore);

/// Holds the control stage for an agent: planners + controllers.
/// `ControlPlugin` calls `step_controllers()` each tick.
#[derive(Component)]
pub struct ControlPipelineComponent(pub ControlCore);

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
