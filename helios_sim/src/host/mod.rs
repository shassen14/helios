//! The shared, reusable Helios simulation host: one definition of "assemble a
//! runnable sim app" that every driver wraps with its own concern (the test
//! harness today, a windowed launcher later).
//!
//! [`HeliosHost`] is a `Plugin` that bundles the host body — windowing, physics,
//! config loading, app state, and the autonomy pipeline — and exposes the few
//! real variation points as constructor inputs (the [`Presentation`] mode).
//! Drivers add it, then layer their own plugins and resources on top:
//!
//! ```ignore
//! App::new()
//!     .add_plugins(HeliosHost::new(cli, Presentation::Headless))
//!     .add_plugins(TestRunnerPlugin) // the driver's own concern
//!     .run();
//! ```

use crate::{
    asset_root, cli::Cli, simulation::config::ConfigPlugin, simulation::core::app_state::AppState,
    HeliosSimulationPlugin,
};

use avian3d::prelude::PhysicsPlugins;
use bevy::app::{PluginGroupBuilder, ScheduleRunnerPlugin};
use bevy::log::{Level, LogPlugin};
use bevy::prelude::*;
use bevy::window::{ExitCondition, WindowPlugin};
use bevy::winit::WinitPlugin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

// The single home for the tracing filter every host shares: keep wgpu's
// chatter at error level while letting the helios crates log at debug. Defined
// once here instead of being re-pasted into each driver bin.
const LOG_FILTER: &str = "info,wgpu_core=error,wgpu_hal=error,helios_sim=debug,helios_core=debug";

// Guards the process-global tracing subscriber. `LogPlugin` installs a logger
// that can only be set once per process, yet a Monte Carlo batch builds a fresh
// `App` — and thus a fresh host — for every run. This flips to `true` the first
// time any host is built; later hosts find it already set and skip `LogPlugin`.
static LOG_INITIALIZED: AtomicBool = AtomicBool::new(false);

pub struct HeliosHost {
    // The resolved launch config. `ConfigPlugin` reads it at startup to locate
    // the scenario file and config root.
    cli: Cli,
    // The one axis the host body varies on: headless vs windowed.
    presentation: Presentation,
}

impl HeliosHost {
    pub fn new(cli: Cli, presentation: Presentation) -> Self {
        Self { cli, presentation }
    }

    fn default_plugins(&self, first: bool) -> PluginGroupBuilder {
        // Start from Bevy's defaults, then replace/disable individual plugins.
        let mut group = DefaultPlugins.build();

        // Pin the asset root to an absolute path so terrain/object GLBs resolve
        // from any working directory — and from a bin in another crate — without
        // needing a `BEVY_ASSET_ROOT` override.
        group = group.set(AssetPlugin {
            file_path: asset_root()
                .into_os_string()
                .into_string()
                .expect("helios_sim asset root is valid UTF-8"),
            ..default()
        });

        // Only the first host in the process owns the global subscriber, so it
        // configures `LogPlugin`; every later host disables it to avoid trying
        // to set the logger twice.
        group = if first {
            group.set(LogPlugin {
                filter: LOG_FILTER.into(),
                level: Level::INFO,
                ..default()
            })
        } else {
            group.disable::<LogPlugin>()
        };

        // Headless: there is no window, so drop `WinitPlugin` (it owns the OS
        // event loop that would normally drive the app) and add a
        // `ScheduleRunnerPlugin` to pump `update()` in its place — without a
        // runner, `run()` ticks once and returns. The windowless `WindowPlugin`
        // plus `DontExit` keeps the app alive despite having no primary window.
        if let Presentation::Headless = self.presentation {
            group = group
                .set(WindowPlugin {
                    primary_window: None,
                    exit_condition: ExitCondition::DontExit,
                    ..default()
                })
                .disable::<WinitPlugin>()
                .add(ScheduleRunnerPlugin::run_loop(Duration::ZERO));
        }

        group
    }
}

impl Plugin for HeliosHost {
    fn build(&self, app: &mut App) {
        // Claim the global-logger slot. `is_ok()` means we won the swap and are
        // the first host built in this process, so we install `LogPlugin`;
        // otherwise `default_plugins` disables it.
        let first = LOG_INITIALIZED
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_ok();

        let group = self.default_plugins(first);

        // Assemble the host body. Order among these is immaterial — each call
        // only registers plugins/resources; the actual work runs later, on the
        // app's first update. `Cli` is inserted for `ConfigPlugin`, which reads
        // it when it loads and resolves the scenario at startup.
        app.add_plugins(group);
        app.add_plugins(PhysicsPlugins::default());

        app.insert_resource(self.cli.clone());
        app.init_state::<AppState>();

        app.add_plugins(ConfigPlugin);

        app.add_plugins(HeliosSimulationPlugin);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Presentation {
    Headless,
    Windowed,
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::PathBuf;

    // A throwaway host. `default_plugins` selects plugins from `presentation`
    // alone and never reads the `Cli` fields, so they only need to be
    // well-formed, not meaningful.
    fn host(presentation: Presentation) -> HeliosHost {
        HeliosHost::new(
            Cli {
                scenario: PathBuf::from("unused.toml"),
                config_root: PathBuf::from("configs"),
                headless: true,
                seed: None,
            },
            presentation,
        )
    }

    #[test]
    fn headless_swaps_winit_for_the_schedule_runner() {
        // Headless has no window, so the OS event loop (Winit) is gone and a
        // ScheduleRunnerPlugin must drive `update()` in its place — without it
        // `run()` would tick once and return before the sim could finish.
        let group = host(Presentation::Headless).default_plugins(true);

        assert!(
            !group.enabled::<WinitPlugin>(),
            "Winit must be disabled in headless mode"
        );
        assert!(
            group.contains::<ScheduleRunnerPlugin>(),
            "headless needs its own runner to pump the loop"
        );
    }

    #[test]
    fn windowed_keeps_winit_and_adds_no_runner() {
        // Windowed keeps Winit as its runner, so a second ScheduleRunnerPlugin
        // would be wrong.
        let group = host(Presentation::Windowed).default_plugins(true);

        assert!(
            group.enabled::<WinitPlugin>(),
            "Winit drives the loop for a windowed host"
        );
        assert!(
            !group.contains::<ScheduleRunnerPlugin>(),
            "windowed must not add a second runner"
        );
    }

    #[test]
    fn the_first_host_in_a_process_installs_the_logger() {
        // `first == true` models the first host built in the process: it owns
        // the one process-global tracing subscriber, so LogPlugin stays enabled.
        let group = host(Presentation::Headless).default_plugins(true);

        assert!(group.enabled::<LogPlugin>());
    }

    #[test]
    fn a_later_host_disables_the_logger() {
        // `first == false` models a subsequent host (e.g. Monte Carlo run > 0):
        // the global subscriber is already set, so LogPlugin must be disabled to
        // avoid the "logger already set" error on every extra run.
        let group = host(Presentation::Headless).default_plugins(false);

        assert!(!group.enabled::<LogPlugin>());
    }
}
