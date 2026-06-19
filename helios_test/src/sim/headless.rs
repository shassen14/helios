use std::time::Duration;

use bevy::{app::ScheduleRunnerPlugin, log::LogPlugin, prelude::*};

/// The plugin set for a windowless test run. Returns the group rather than an
/// `App` so the *bin* owns the `App` and decides what else to add
/// (`HeliosSimulationPlugin`, the runner resources, `TestRunnerPlugin`).
pub fn headless() -> impl PluginGroup {
    DefaultPlugins
        // No window: a test run renders nothing, and CI has no display.
        .set(WindowPlugin {
            primary_window: None,
            // Without a window the app would otherwise exit immediately;
            // `DontExit` keeps the loop alive so the runner can tick.
            exit_condition: bevy::window::ExitCondition::DontExit,
            ..default()
        })
        // Winit drives the OS event loop for a real window — useless headless,
        // and it would try to grab a display we don't have.
        .disable::<bevy::winit::WinitPlugin>()
        // Keep the console quiet so the test signal is the report, not log
        // spam; wgpu's chatter is demoted to errors only.
        .set(LogPlugin {
            level: bevy::log::Level::INFO,
            filter: "info,wgpu_core=error,wgpu_hal=error,helios_sim=debug,helios_core=debug"
                .to_string(),
            ..default()
        })
        // With Winit gone there is no event loop to drive the app, so an
        // unconfigured `run()` would tick once and return. ScheduleRunnerPlugin
        // supplies a headless loop that pumps `update()` until an `AppExit` is
        // sent (which `finalize` does once the run terminates). `Duration::ZERO`
        // = run as fast as possible; simulated time still tracks wall time.
        .add(ScheduleRunnerPlugin::run_loop(Duration::ZERO))
}
