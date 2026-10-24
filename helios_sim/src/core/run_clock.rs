//! The run's clock: simulated time since the run started, the one time every
//! stamp on an agent's bus and every lookup against its buffers reads.
//!
//! Bevy's clocks count from app start, loading included, and loading lasts
//! as long as its asset work does. A stamp read from them carries that
//! wall-time dependent origin, which rounds every later value differently in
//! its last bits. So the origin is taken once, as the app enters
//! [`AppState::Running`], and readers get the exact `Duration` since then.
//!
//! Reading Bevy's elapsed time anywhere else in the crate is refused by
//! clippy (`disallowed-methods` in the workspace `clippy.toml`).
//!
//! The run's start is also when physics starts. Before it, bodies spawned
//! while the scene builds would fall for however many fixed steps the frame
//! that spawns them runs, so the state a run starts from would depend on the
//! frame. Physics is paused from startup and unpaused with the clock, so
//! `t = 0` is the spawn state.

use crate::prelude::*;

use avian3d::prelude::{Physics, PhysicsTime};
use bevy::ecs::system::SystemParam;
use std::time::Duration;

/// Starts the run's clock, and physics with it, as the app enters
/// [`AppState::Running`].
pub struct RunClockPlugin;

impl Plugin for RunClockPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, pause_physics);
        app.add_systems(OnEnter(AppState::Running), (start_run_clock, start_physics));
    }
}

/// The run's simulated time.
///
/// In `FixedUpdate` it is the step being run: the run's `k`-th step reads
/// exactly `k` timesteps, and physics has already moved the world to it.
/// Elsewhere it is the last step the frame ran, which is also the brain's
/// newest tick. Before the run starts there is no run time, so both reads
/// give `None`.
#[derive(SystemParam)]
pub struct RunClock<'w> {
    start: Option<Res<'w, RunStart>>,
    fixed: Res<'w, Time<Fixed>>,
}

impl RunClock<'_> {
    /// The instant the run starts at: run time counts from here.
    pub const START: MonotonicTime = MonotonicTime(0.0);

    /// The time since the run started.
    pub fn elapsed(&self) -> Option<Duration> {
        let start = self.start.as_deref()?;
        // The one sanctioned read: the difference of two fixed-clock instants
        // is an exact integer count of nanoseconds, whatever the origin.
        #[allow(clippy::disallowed_methods)]
        let fixed_now = self.fixed.elapsed();
        Some(fixed_now.saturating_sub(start.0))
    }

    /// The run's current instant, as a bus stamp carries it.
    pub fn now(&self) -> Option<MonotonicTime> {
        self.elapsed()
            .map(|elapsed| MonotonicTime(elapsed.as_secs_f64()))
    }
}

/// The fixed clock's instant as the run started. Absent before.
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
struct RunStart(Duration);

/// Runs in the state transition, before the frame's fixed loop, so the run's
/// first step is one timestep past the start.
fn start_run_clock(mut commands: Commands, fixed: Res<Time<Fixed>>) {
    #[allow(clippy::disallowed_methods)]
    let start = fixed.elapsed();
    commands.insert_resource(RunStart(start));
}

/// Runs at startup, before any fixed step, whatever order plugins were added
/// in.
fn pause_physics(mut physics: ResMut<Time<Physics>>) {
    physics.pause();
}

fn start_physics(mut physics: ResMut<Time<Physics>>) {
    physics.unpause();
}

/// Starts the run's clock at the fixed clock's current instant, for a test
/// world that has no state machine.
#[cfg(test)]
pub(crate) fn start_run_clock_now(world: &mut World) {
    #[allow(clippy::disallowed_methods)]
    let start = world.resource::<Time<Fixed>>().elapsed();
    world.insert_resource(RunStart(start));
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::ecs::system::RunSystemOnce;
    use bevy::state::app::StatesPlugin;

    /// The scenario's 400 Hz tick.
    const STEP: Duration = Duration::from_micros(2500);
    /// Loading lengths seen on an idle machine and under load: 21 and 41
    /// steps of fixed time before the run starts.
    const IDLE_ORIGIN: Duration = Duration::from_micros(52_500);
    const LOADED_ORIGIN: Duration = Duration::from_micros(102_500);
    /// Steps per run: a minute at 400 Hz.
    const STEPS: u32 = 24_000;

    /// An app whose fixed clock has advanced by `origin` while loading.
    fn app_loaded_for(origin: Duration) -> App {
        let mut app = App::new();
        app.add_plugins((StatesPlugin, RunClockPlugin));
        app.init_state::<AppState>();
        app.init_resource::<Time<Physics>>();

        let mut fixed = Time::<Fixed>::from_duration(STEP);
        fixed.advance_by(origin);
        app.insert_resource(fixed);
        app
    }

    fn enter_running(app: &mut App) {
        app.world_mut()
            .resource_mut::<NextState<AppState>>()
            .set(AppState::Running);
        app.update();
    }

    fn step(app: &mut App) {
        app.world_mut()
            .resource_mut::<Time<Fixed>>()
            .advance_by(STEP);
    }

    fn now(app: &mut App) -> Option<MonotonicTime> {
        app.world_mut()
            .run_system_once(|clock: RunClock| clock.now())
            .expect("the clock reads")
    }

    /// Every stamp of a run, one per step, read after the fixed clock has
    /// advanced by `origin` while loading.
    fn stamps_after_loading_for(origin: Duration) -> Vec<MonotonicTime> {
        let mut app = app_loaded_for(origin);
        enter_running(&mut app);
        (0..STEPS)
            .map(|_| {
                step(&mut app);
                now(&mut app).expect("the run has started")
            })
            .collect()
    }

    #[test]
    fn there_is_no_run_time_before_the_run_starts() {
        let mut app = app_loaded_for(IDLE_ORIGIN);
        app.update();

        assert_eq!(now(&mut app), None);
    }

    #[test]
    fn the_runs_kth_step_reads_k_timesteps_bit_for_bit() {
        let stamps = stamps_after_loading_for(LOADED_ORIGIN);

        for (k, stamp) in (1..=STEPS).zip(stamps) {
            let expected = (STEP * k).as_secs_f64();
            assert_eq!(stamp.0.to_bits(), expected.to_bits(), "step {k}");
        }
    }

    /// The property the clock exists for: how long loading took changes no
    /// stamp. The fixed clock's own seconds do differ in their last bits.
    #[test]
    fn loading_time_changes_no_stamp() {
        assert_eq!(
            stamps_after_loading_for(IDLE_ORIGIN),
            stamps_after_loading_for(LOADED_ORIGIN)
        );

        let fixed_seconds =
            |origin: Duration, k: u32| (origin + STEP * k).as_secs_f64() - origin.as_secs_f64();
        let differs =
            (1..=STEPS).any(|k| fixed_seconds(IDLE_ORIGIN, k) != fixed_seconds(LOADED_ORIGIN, k));
        assert!(differs, "the guard needs origins whose rounding differs");
    }
}
