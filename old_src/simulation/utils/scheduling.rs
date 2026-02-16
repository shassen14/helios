use crate::simulation::core::config::SimulationConfig;
use bevy::prelude::*;
use std::time::Duration;

/// A generic Bevy run condition that runs a system at a specific frequency.
/// The frequency is determined by the `rate_extractor` function, which pulls
/// the correct rate (in Hz) from the global `SimulationConfig` resource.
pub fn run_at_configured_rate<F>(
    config: Res<SimulationConfig>,
    time: Res<Time>,
    mut timer: Local<Timer>,
    // The rate_extractor is a function that takes the config and returns the desired rate.
    // It must be a system parameter (`SystemParam`) to be used here.
    rate_extractor: F,
) -> bool
where
    // The `F` parameter is a function that takes no arguments and returns an Option<f32>.
    // This looks a bit magical, but it's how Bevy allows systems to be passed as parameters.
    F: Fn() -> Option<f32> + 'static,
{
    // Get the rate using the provided extractor function.
    if let Some(rate_hz) = rate_extractor() {
        if rate_hz <= 0.0 {
            return false;
        } // Don't run if rate is zero or negative.

        let target_duration = Duration::from_secs_f32(1.0 / rate_hz);
        if timer.duration() != target_duration {
            *timer = Timer::new(target_duration, TimerMode::Repeating);
        }

        timer.tick(time.delta());
        timer.just_finished()
    } else {
        // If the extractor returns None, it means the config for this system doesn't exist.
        // For example, planning an agent that has no planner config.
        false
    }
}
