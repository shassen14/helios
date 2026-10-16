use super::{ActiveRunner, RunOutcome, RunVerdict, WallClockStart};
use crate::run::termination::TerminationReason;
use crate::sim::{ReportOutputPath, RunMetadata};
use crate::{HostFacts, ObservationCollector, ReportStatus, Runner, TickAction};

use helios_core::prelude::{AgentId, MonotonicTime};
use helios_runtime::{port::RefusedWrites, AutonomyPipeline};
use helios_sim::brain_bridge::{
    components::{AgentIdComponent, AutonomyPipelineComponent, PipelineBuildFailed},
    ObservationSinkAppExt, RegisteredSink,
};
use helios_sim::prelude::{AppState, SimulationSet};

use bevy::prelude::*;

/// Sets up one test run on an [`App`].
pub trait TestRunAppExt {
    /// Holds `runner`, registers the collector it reads as an observation
    /// sink, and adds the systems that set it up, tick it and finalize it.
    /// The runner and its collector are only ever added together, so the
    /// systems always find both and both describe the same run.
    fn add_test_run(&mut self, runner: Runner) -> &mut Self;
}

impl TestRunAppExt for App {
    fn add_test_run(&mut self, runner: Runner) -> &mut Self {
        // Built before the runner moves into the world.
        let collector = runner.collector();

        self.add_observation_sink(collector);
        self.insert_resource(ActiveRunner::new(runner));
        self.add_plugins(TestRunnerPlugin);

        self
    }
}

/// The three runner systems. Added only by [`TestRunAppExt::add_test_run`],
/// which also supplies the runner and collector they read.
struct TestRunnerPlugin;

impl Plugin for TestRunnerPlugin {
    fn build(&self, app: &mut App) {
        // on_pipeline_built_system and finalize system run only once
        // tick system runs during the simulation and will need a set to be in for run
        app.add_systems(OnEnter(AppState::Running), on_pipeline_built_system)
            .add_systems(
                FixedUpdate,
                tick_system
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(OnEnter(AppState::Flushing), finalize_system);
    }
}

/// System 1 — runs once at `OnEnter(AppState::Running)`, after the scene is
/// built and every agent's pipeline exists. Hands the runner the per-agent
/// pipelines and the run's collector, so it can check that every assertion
/// can be judged.
///
/// Two setup failures share one exit here, both legitimate **test failures**
/// (exit 1) rather than crashes: an agent whose pipeline could not be
/// assembled at all, and an assertion the run can't judge (a target no agent
/// reports, a payload with no extractor, a nondeterministic leaf asserted on
/// without permission).
/// Either marks the verdict failed and routes to `Flushing`, so both flow
/// through the normal report path.
///
/// The unbuilt-pipeline check has to come first, and cannot be folded into
/// the runner: an agent with no pipeline is absent from `pairs` entirely, so
/// the runner sees a scenario that merely has fewer agents than the author
/// intended — and with no assertions naming it, nothing to complain about.
fn on_pipeline_built_system(
    query: Query<(&AgentIdComponent, &AutonomyPipelineComponent)>,
    failed_query: Query<(&AgentIdComponent, &PipelineBuildFailed)>,
    collector: Res<RegisteredSink<ObservationCollector>>,
    mut runner: ResMut<ActiveRunner>,
    mut next_state: ResMut<NextState<AppState>>,
    mut run_verdict: ResMut<RunVerdict>,
) {
    if !failed_query.is_empty() {
        for (agent_id_comp, failure) in failed_query.iter() {
            for err in &failure.errors {
                let reason = format!(
                    "agent '{}' has no autonomy pipeline: {}",
                    agent_id_comp.0, err
                );
                tracing::error!("{reason}");
                runner.0.fail_setup(reason);
            }
        }
        run_verdict.set(ReportStatus::Failed);
        next_state.set(AppState::Flushing);
        return;
    }

    // The query returns only agents with BOTH components — guaranteed by the
    // co-located insert in spawn_autonomy_pipeline (see AgentIdComponent docs).
    let mut pairs: Vec<(AgentId, &AutonomyPipeline)> = Vec::new();
    for (agent_id_comp, autonomy_pipeline_comp) in query.iter() {
        pairs.push((agent_id_comp.0.clone(), &autonomy_pipeline_comp.0));
    }

    match runner.0.on_pipeline_built(&pairs, &collector.0) {
        Ok(()) => (),
        Err(e) => {
            let reason = format!("on_pipeline_built failed: {e}");
            tracing::error!("{reason}");
            runner.0.fail_setup(reason);
            run_verdict.set(ReportStatus::Failed);
            next_state.set(AppState::Flushing);
        }
    }
}

/// System 2 — runs every `FixedUpdate` in `SimulationSet::Validation`, which is
/// ordered after `SimulationSet::Observation`, so the collector already holds
/// what the brain reported this tick when the runner judges it.
///
/// The runner owns termination (via the run file's `termination` block); this
/// system only honors the `Terminate` it is handed by stashing the reason and
/// routing to `Flushing`. No host-side timeout lives here.
fn tick_system(
    collector: Res<RegisteredSink<ObservationCollector>>,
    time: Res<Time<Fixed>>,
    mut runner: ResMut<ActiveRunner>,
    mut outcome: ResMut<RunOutcome>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    // The run is already over. Bevy's fixed loop runs a whole batch of steps
    // per frame and cannot be broken out of partway, so the `Flushing` request
    // made below doesn't take effect until the batch drains — leaving trailing
    // steps that advance physics past the run's own stopping point. Observing
    // them would judge continuous assertions against simulated time the run
    // file never asked for, so the runner sits them out.
    if outcome.is_terminated() {
        return;
    }

    // Same clock source as the brain's tick, so the time budget and the brain
    // never disagree.
    let now = MonotonicTime(time.elapsed_secs_f64());

    match runner.0.tick(now, &collector.0) {
        TickAction::Terminate { reason } => {
            outcome.terminate(now, reason);
            next_state.set(AppState::Flushing);
        }
        TickAction::Continue => (),
    }
}

/// System 3 — runs once at `OnEnter(AppState::Flushing)`. Builds the final
/// `Report` (terminal assertions judged now; continuous ones already folded
/// across ticks into their latches, every agent's refused writes read from
/// its pipeline's bus), writes it to disk and the console, records
/// the verdict, and ends the Bevy loop. Takes the runner as `Res` because
/// `finalize` only reads.
// Bevy systems declare their dependencies as parameters, so the count exceeds
// clippy's default threshold; that's idiomatic here, not a smell.
#[allow(clippy::too_many_arguments)]
fn finalize_system(
    collector: Res<RegisteredSink<ObservationCollector>>,
    // `Time<Fixed>`, not the generic `Time`. This system runs on a state
    // transition, which the frame orders *before* the fixed loop — so generic
    // `Time` here resolves to `Time<Virtual>` and reads a moment up to one
    // frame past the tick the assertions were actually judged on.
    time: Res<Time<Fixed>>,
    wall_start: Res<WallClockStart>,
    runner: Res<ActiveRunner>,
    outcome: Res<RunOutcome>,
    out_path: Res<ReportOutputPath>,
    metadata: Res<RunMetadata>,
    pipelines: Query<(&AgentIdComponent, &AutonomyPipelineComponent)>,
    mut verdict: ResMut<RunVerdict>,
    mut exit: MessageWriter<AppExit>,
) {
    // The tick that ended the run, not the clock as it stands now — the two
    // differ by however many steps the fixed loop had left in its batch. Falls
    // back to the live clock only on the setup-failure path, where no tick ever
    // ran to record one.
    let now = outcome
        .terminated_at
        .unwrap_or_else(|| MonotonicTime(time.elapsed_secs_f64()));
    let wall_secs = wall_start.0.elapsed().as_secs_f64();
    let run_name = outcome.run_name.clone();
    // `None` only on the "terminated before any tick" path (system 1's
    // short-circuit): setup never produced a tick, so there is no real
    // termination cause. `Aborted` names that case instead of borrowing a
    // misleading stand-in like the time budget.
    let reason = outcome.reason.clone().unwrap_or(TerminationReason::Aborted);

    // Sorted by agent, since query order is not stable across runs and the
    // report must be.
    let mut refused_writes: Vec<(AgentId, RefusedWrites)> = pipelines
        .iter()
        .flat_map(|(agent, pipeline)| {
            pipeline
                .0
                .bus()
                .refused_writes()
                .into_iter()
                .map(|refused| (agent.0.clone(), refused))
        })
        .collect();
    refused_writes.sort_by_cached_key(|(agent, _)| agent.to_string());

    let host = HostFacts {
        sim_now: now,
        wall_duration_secs: wall_secs,
        run_name,
        terminated_by_reason: reason,
        master_seed: metadata.seed,
        refused_writes,
    };
    let report = runner.0.finalize(host, &collector.0);

    // Two surfaces: machine-readable TOML for CI, human summary for the console.
    // A failed write is logged, not panicked — runtime paths never unwrap.
    if let Err(e) = crate::report::toml_writer::write(&report, &out_path.0) {
        tracing::error!("failed to write report TOML: {e}");
    }
    crate::report::console::print(&report);

    verdict.set(report.status().clone());

    // Carry the verdict out as the app's exit code. `app.run()` empties the App
    // (Bevy 0.16), so the bin cannot read RunVerdict from the world afterward —
    // it reads the `AppExit` that `run()` returns instead. Success → exit 0,
    // code 1 → exit 1; the bin maps that to the process exit status.
    let exit_event = if verdict.passed() {
        AppExit::Success
    } else {
        AppExit::from_code(1)
    };
    exit.write(exit_event);
}
