//! The shared, reusable Helios simulation host: one definition of "assemble a
//! runnable sim app" that every driver wraps with its own concern (the test
//! harness today, a windowed launcher later).
//!
//! [`HeliosHost`] is a `Plugin` that bundles the host body — windowing, physics,
//! config loading, app state, and the autonomy pipeline — and exposes the few
//! real variation points as constructor inputs ([`Presentation`], which decides
//! whether there is a window, and [`TimePolicy`], which decides how fast the
//! clock runs). Drivers add it, then layer their own plugins and resources on
//! top:
//!
//! ```ignore
//! App::new()
//!     .add_plugins(HeliosHost::new(
//!         cli,
//!         Presentation::Headless,
//!         TimePolicy::FastAsPossible,
//!     ))
//!     .add_plugins(TestRunnerPlugin) // the driver's own concern
//!     .run();
//! ```

use crate::{
    asset_root, cli::Cli, config::ConfigPlugin, core::app_state::AppState, HeliosSimulationPlugin,
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
    // Whether the host body is headless or windowed. Selects plugins.
    presentation: Presentation,
    // How simulated time is paced. Independent of `presentation` — it selects
    // no plugins, only how the clock those plugins share is driven.
    time_policy: TimePolicy,
}

impl HeliosHost {
    pub fn new(cli: Cli, presentation: Presentation, time_policy: TimePolicy) -> Self {
        Self {
            cli,
            presentation,
            time_policy,
        }
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

        // The policy is applied here *and* published as a resource: the clamp
        // budget it implies depends on the tick rate, which only exists once
        // the scenario TOML is loaded, so `configure_time_pacing` finishes the
        // job at kickoff.
        app.insert_resource(self.time_policy);
        apply_time_policy(app, self.time_policy);

        app.add_plugins(ConfigPlugin);

        app.add_plugins(HeliosSimulationPlugin);
    }
}

/// Point the app's clock at the chosen pacing. Safe to call during
/// `Plugin::build` because `DefaultPlugins` has already brought `TimePlugin`,
/// and neither branch depends on the scenario file.
fn apply_time_policy(app: &mut App, policy: TimePolicy) {
    match policy {
        // Bevy's out-of-the-box behavior; nothing to override.
        TimePolicy::RealTime => (),

        TimePolicy::Scaled(factor) => {
            // Checked here rather than left to `set_relative_speed`, whose own
            // panics ("tried to go back in time") don't mention the flag that
            // caused them. Zero is rejected too: a frozen clock never reaches
            // the run's time budget, so the sim would hang rather than fail.
            assert!(
                factor.is_finite() && factor > 0.0,
                "--speed must be a finite value greater than zero, got {factor}"
            );
            app.world_mut()
                .resource_mut::<Time<Virtual>>()
                .set_relative_speed(factor);
        }

        // Nothing to do here. Detaching the clock from the wall takes a batch
        // size derived from the tick rate, which the scenario file carries and
        // which is not loaded yet — `configure_time_pacing` does it at kickoff.
        TimePolicy::FastAsPossible => (),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Presentation {
    Headless,
    Windowed,
}

/// How simulated time is paced against the wall clock. Deliberately orthogonal
/// to [`Presentation`]: fast-forward is as useful in a window — watching a long
/// scenario without waiting for it — as it is headless.
///
/// No variant changes what a run *computes*. The fixed timestep is identical
/// under all three, so the sim takes the same steps and produces the same
/// numbers; only the delay before you see them differs.
// `Eq` is absent on purpose: `Scaled` carries an `f32`.
#[derive(Resource, Clone, Copy, Debug, PartialEq)]
pub enum TimePolicy {
    /// One second of simulated time per second of wall time.
    RealTime,
    /// Wall-paced but scaled: `n` seconds of simulated time per wall second.
    /// The machine still has to do the work, so a factor it cannot sustain
    /// degrades to "as fast as it manages" rather than skipping simulation.
    Scaled(f32),
    /// The wall clock leaves the loop entirely: simulated time advances by the
    /// fixed timestep once per `App::update`, so a run finishes as fast as the
    /// math allows. The pacing a test harness wants.
    FastAsPossible,
}

impl TimePolicy {
    /// The policy implied by the launch flags. An explicit `--speed` always
    /// wins; otherwise the presentation decides, because the two exist for
    /// different reasons. A window is there to be watched, so it defaults to
    /// real time; a headless run is there to produce an answer, so it defaults
    /// to reaching one as quickly as it can.
    pub fn from_cli(speed: Option<f32>, presentation: Presentation) -> Self {
        match (speed, presentation) {
            (Some(factor), _) => TimePolicy::Scaled(factor),
            (None, Presentation::Windowed) => TimePolicy::RealTime,
            (None, Presentation::Headless) => TimePolicy::FastAsPossible,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::PathBuf;

    use bevy::time::TimeUpdateStrategy;

    // A throwaway host. `default_plugins` selects plugins from `presentation`
    // alone and reads neither the `Cli` fields nor the time policy, so those
    // only need to be well-formed, not meaningful.
    fn host(presentation: Presentation) -> HeliosHost {
        HeliosHost::new(
            Cli {
                scenario: PathBuf::from("unused.toml"),
                config_root: PathBuf::from("configs"),
                headless: true,
                speed: None,
                seed: None,
            },
            presentation,
            TimePolicy::RealTime,
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

    // A minimal app carrying only the clock, so these assert what
    // `apply_time_policy` did and nothing about the rest of the host body.
    fn timed_app(policy: TimePolicy) -> App {
        let mut app = App::new();
        app.add_plugins(bevy::time::TimePlugin);
        apply_time_policy(&mut app, policy);
        app
    }

    #[test]
    fn real_time_leaves_bevys_own_pacing_untouched() {
        let app = timed_app(TimePolicy::RealTime);

        assert_eq!(
            app.world().resource::<Time<Virtual>>().relative_speed(),
            1.0
        );
        assert!(matches!(
            app.world().resource::<TimeUpdateStrategy>(),
            TimeUpdateStrategy::Automatic
        ));
    }

    #[test]
    fn scaled_speeds_up_the_virtual_clock_but_keeps_it_wall_paced() {
        // The clock still reads the wall clock — it just multiplies what it
        // finds there — so the update strategy must stay `Automatic`.
        let app = timed_app(TimePolicy::Scaled(10.0));

        assert_eq!(
            app.world().resource::<Time<Virtual>>().relative_speed(),
            10.0
        );
        assert!(matches!(
            app.world().resource::<TimeUpdateStrategy>(),
            TimeUpdateStrategy::Automatic
        ));
    }

    #[test]
    fn fast_as_possible_defers_to_kickoff() {
        // Detaching the clock needs a batch size derived from the tick rate,
        // and the scenario file carrying it is not loaded during `build`. The
        // clock must therefore still be untouched here — `configure_time_pacing`
        // owns this policy, and a value set now would only be a stale guess.
        let app = timed_app(TimePolicy::FastAsPossible);

        assert!(matches!(
            app.world().resource::<TimeUpdateStrategy>(),
            TimeUpdateStrategy::Automatic
        ));
    }

    #[test]
    #[should_panic(expected = "--speed")]
    fn a_zero_speed_is_refused_rather_than_freezing_the_clock() {
        // A stopped clock never reaches the run's time budget, so accepting
        // this would hang the run instead of failing it.
        let _ = timed_app(TimePolicy::Scaled(0.0));
    }

    #[test]
    fn an_explicit_speed_wins_over_the_presentation_default() {
        // `--speed` is the author saying what they want; neither presentation
        // may quietly substitute its own preference for it.
        assert_eq!(
            TimePolicy::from_cli(Some(10.0), Presentation::Headless),
            TimePolicy::Scaled(10.0)
        );
        assert_eq!(
            TimePolicy::from_cli(Some(10.0), Presentation::Windowed),
            TimePolicy::Scaled(10.0)
        );
    }

    #[test]
    fn a_window_defaults_to_real_time() {
        // A window exists to be watched, and a run that races past at CPU speed
        // shows nothing a human can follow.
        assert_eq!(
            TimePolicy::from_cli(None, Presentation::Windowed),
            TimePolicy::RealTime
        );
    }

    #[test]
    fn a_headless_run_defaults_to_finishing_as_fast_as_it_can() {
        // Nobody is watching, so pacing to the wall clock buys nothing and
        // costs a full scenario duration per run.
        assert_eq!(
            TimePolicy::from_cli(None, Presentation::Headless),
            TimePolicy::FastAsPossible
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
