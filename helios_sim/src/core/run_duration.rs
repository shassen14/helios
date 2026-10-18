//! Ends a run once the scenario's `[simulation] duration_seconds` of simulated
//! time has passed since it started running.
//!
//! Only a driver with no other way to stop adds this: a headless
//! `helios_play` has no window to close. A windowed session ends when its
//! viewer closes the window, and the test harness ends a run by its own run
//! file's termination rules.

use crate::prelude::*;

use std::time::Duration;

/// Moves the app to [`AppState::Flushing`] once the scenario's duration has
/// been simulated, then exits it.
pub struct RunDurationPlugin;

impl Plugin for RunDurationPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(AppState::Running), start_run_clock)
            .add_systems(
                FixedUpdate,
                end_run_at_deadline
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(OnEnter(AppState::Flushing), exit_app);
    }
}

/// The fixed-clock time at which the run ends.
///
/// Measured from when the scene starts running, not from app start, so time
/// spent loading assets and building the scene doesn't eat into the run.
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
struct RunDeadline(Duration);

/// Sets the deadline from the scenario's duration and the clock as the run
/// starts.
fn start_run_clock(mut commands: Commands, config: Res<ScenarioConfig>, time: Res<Time<Fixed>>) {
    let duration = run_duration(config.common.simulation.duration_seconds);
    info!(
        "run ends after {:.1} s of simulated time",
        duration.as_secs_f64()
    );
    commands.insert_resource(RunDeadline(time.elapsed() + duration));
}

/// Ends the run on the first tick at or past the deadline.
fn end_run_at_deadline(
    deadline: Option<Res<RunDeadline>>,
    time: Res<Time<Fixed>>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    // Inserted on entering `Running`, so it is always present here; a missing
    // one keeps the run going rather than panicking mid-run.
    let Some(deadline) = deadline else {
        return;
    };
    if time.elapsed() >= deadline.0 {
        next_state.set(AppState::Flushing);
    }
}

fn exit_app(mut exit: MessageWriter<AppExit>) {
    info!("run reached its duration; exiting");
    exit.write(AppExit::Success);
}

/// The run length a scenario's `duration_seconds` asks for.
///
/// Rejects a non-positive or non-finite value at startup, naming the field,
/// rather than ending the run on its first tick or never ending it.
fn run_duration(seconds: f32) -> Duration {
    assert!(
        seconds.is_finite() && seconds > 0.0,
        "[simulation] duration_seconds must be a finite value greater than zero, got {seconds}"
    );
    Duration::from_secs_f64(f64::from(seconds))
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::ecs::system::RunSystemOnce;
    use bevy::state::app::StatesPlugin;

    /// An app holding only the state machine, the fixed clock advanced by
    /// `elapsed`, and a deadline.
    fn app_at(elapsed: Duration, deadline: Duration) -> App {
        let mut app = App::new();
        app.add_plugins(StatesPlugin);
        app.init_state::<AppState>();

        let mut time = Time::<Fixed>::default();
        time.advance_by(elapsed);
        app.insert_resource(time);
        app.insert_resource(RunDeadline(deadline));
        app
    }

    fn flushing_requested(app: &App) -> bool {
        matches!(
            app.world().resource::<NextState<AppState>>(),
            NextState::Pending(AppState::Flushing)
        )
    }

    #[test]
    fn the_run_continues_before_its_deadline() {
        let mut app = app_at(Duration::from_secs(9), Duration::from_secs(10));
        app.world_mut()
            .run_system_once(end_run_at_deadline)
            .expect("system runs");

        assert!(!flushing_requested(&app));
    }

    #[test]
    fn the_run_ends_at_its_deadline() {
        let mut app = app_at(Duration::from_secs(10), Duration::from_secs(10));
        app.world_mut()
            .run_system_once(end_run_at_deadline)
            .expect("system runs");

        assert!(flushing_requested(&app));
    }

    #[test]
    fn a_positive_duration_is_taken_as_seconds() {
        assert_eq!(run_duration(2.5), Duration::from_millis(2500));
    }

    #[test]
    #[should_panic(expected = "duration_seconds")]
    fn a_zero_duration_is_refused() {
        // It would end the run on its first tick, before anything happened.
        run_duration(0.0);
    }

    #[test]
    #[should_panic(expected = "duration_seconds")]
    fn a_non_finite_duration_is_refused() {
        run_duration(f32::NAN);
    }
}
