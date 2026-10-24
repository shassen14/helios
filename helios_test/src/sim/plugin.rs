use super::{ActiveRunner, RunOutcome, RunVerdict, WallClockStart};
use crate::runner::UnbuiltPipeline;
use crate::sim::{ReportOutputPath, ReportSlot, RunSeed};
use crate::{HostFacts, ObservationCollector, Runner, TickAction};

use helios_core::prelude::AgentId;
use helios_runtime::{port::RefusedWrites, AutonomyPipeline};
use helios_sim::brain_bridge::{
    components::{AgentIdComponent, AutonomyPipelineComponent, PipelineBuildFailed},
    ObservationSinkAppExt, RegisteredSink,
};
use helios_sim::prelude::{AppState, RunClock, SceneBuildSet, SimulationSet};

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
        // Setup runs once at scene build, where the host then asks the
        // collector what to record; an aborted run stops when Running starts,
        // since scene build's last pass sets Running itself. The tick runs
        // through the simulation, finalize once at its end.
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            on_pipeline_built_system.in_set(SceneBuildSet::SetUpSinks),
        )
        .add_systems(OnEnter(AppState::Running), stop_aborted_run_system)
        .add_systems(
            FixedUpdate,
            tick_system
                .in_set(SimulationSet::Validation)
                .run_if(in_state(AppState::Running)),
        )
        .add_systems(OnEnter(AppState::Flushing), finalize_system);
    }
}

/// System 1 — runs once at scene build, in `SceneBuildSet::SetUpSinks`,
/// after every agent's pipeline exists or has failed to build, and before
/// the host asks the collector what to record. Hands the runner the
/// per-agent pipelines, the agents whose pipeline failed with their errors,
/// and the run's collector, so it can check that every assertion can be
/// judged and bind what the collector records.
///
/// An agent with no pipeline must be passed, not left out: absent, the
/// runner would see a scenario that merely has fewer agents than the author
/// intended — and with no assertions naming it, nothing to complain about.
/// A refusal aborts the runner, which records why for its report;
/// [`stop_aborted_run_system`] routes it through the normal report path.
/// Here it is only logged.
fn on_pipeline_built_system(
    query: Query<(&AgentIdComponent, &AutonomyPipelineComponent)>,
    failed_query: Query<(&AgentIdComponent, &PipelineBuildFailed)>,
    mut collector: ResMut<RegisteredSink<ObservationCollector>>,
    mut runner: ResMut<ActiveRunner>,
) {
    // The query returns only agents with BOTH components — guaranteed by the
    // co-located insert in spawn_autonomy_pipeline (see AgentIdComponent docs).
    let mut pairs: Vec<(AgentId, &AutonomyPipeline)> = Vec::new();
    for (agent_id_comp, autonomy_pipeline_comp) in query.iter() {
        pairs.push((agent_id_comp.0.clone(), &autonomy_pipeline_comp.0));
    }
    // Sorted by agent, since query order is not stable across runs and the
    // report lists them in this order.
    let mut unbuilt: Vec<UnbuiltPipeline> = failed_query
        .iter()
        .map(|(agent, failure)| UnbuiltPipeline {
            agent: agent.0.clone(),
            errors: failure.errors.clone(),
        })
        .collect();
    unbuilt.sort_by_cached_key(|unbuilt| unbuilt.agent.to_string());

    if let Err(e) = runner
        .0
        .on_pipeline_built(&pairs, &unbuilt, &mut collector.0)
    {
        tracing::error!("the run can't be set up: {e}");
    }
}

/// Runs once at `OnEnter(AppState::Running)`: a run whose setup aborted
/// routes to `Flushing`, so it reports without ticking. Not done at setup,
/// in scene build: its last pass sets `Running` after, and the later
/// transition wins.
fn stop_aborted_run_system(runner: Res<ActiveRunner>, mut next_state: ResMut<NextState<AppState>>) {
    if runner.0.is_aborted() {
        next_state.set(AppState::Flushing);
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
    clock: RunClock,
    mut runner: ResMut<ActiveRunner>,
    mut outcome: ResMut<RunOutcome>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    // The run is already over. Bevy's fixed loop runs a whole batch of steps
    // per frame and cannot be broken out of partway, so the `Flushing` request
    // made below doesn't take effect until the batch drains — leaving trailing
    // steps that advance physics past the run's own stopping point. Observing
    // them would judge every-sample assertions against simulated time the run
    // file never asked for, so the runner sits them out.
    if outcome.is_terminated() {
        return;
    }

    // Same clock as the brain's tick, so the time budget and the brain never
    // disagree. Runs only while `Running`, when the clock has started.
    let Some(now) = clock.now() else {
        return;
    };

    match runner.0.tick(now, &collector.0) {
        TickAction::Terminate { reason } => {
            outcome.terminate(now, Some(reason));
            next_state.set(AppState::Flushing);
        }
        TickAction::Abort => {
            outcome.terminate(now, None);
            next_state.set(AppState::Flushing);
        }
        TickAction::Continue => (),
    }
}

/// System 3 — runs once at `OnEnter(AppState::Flushing)`. Builds the final
/// `Report` (reduced assertions and measures evaluated now; every-sample ones
/// already folded across ticks into their latches, every agent's refused
/// writes read from its pipeline's bus), writes it to disk and the console,
/// records the verdict, hands the report to the bin through its slot, and
/// ends the Bevy loop. Takes the runner as `Res` because
/// `finalize` only reads.
// Bevy systems declare their dependencies as parameters, so the count exceeds
// clippy's default threshold; that's idiomatic here, not a smell.
#[allow(clippy::too_many_arguments)]
fn finalize_system(
    collector: Res<RegisteredSink<ObservationCollector>>,
    clock: RunClock,
    wall_start: Res<WallClockStart>,
    runner: Res<ActiveRunner>,
    outcome: Res<RunOutcome>,
    out_path: Res<ReportOutputPath>,
    seed: Res<RunSeed>,
    slot: Res<ReportSlot>,
    pipelines: Query<(&AgentIdComponent, &AutonomyPipelineComponent)>,
    mut verdict: ResMut<RunVerdict>,
    mut exit: MessageWriter<AppExit>,
) {
    // The tick that ended the run, not the clock as it stands now — the two
    // differ by however many steps the fixed loop had left in its batch. Falls
    // back to the live clock only on the setup-failure path, where no tick ever
    // ran to record one, and to the run's start if the run never started.
    let now = outcome
        .terminated_at
        .or_else(|| clock.now())
        .unwrap_or(RunClock::START);
    let wall_secs = wall_start.0.elapsed().as_secs_f64();
    let run_name = outcome.run_name.clone();
    // `None` when the runner aborted the run, which it explains itself.
    let reason = outcome.reason.clone();

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
        master_seed: seed.0,
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
    // Last, since storing moves it: a batch aggregates it after `run()`.
    slot.store(report);

    // Carry the verdict out as the app's exit code. `app.run()` empties the App
    // (Bevy 0.16), so the bin cannot read RunVerdict from the world afterward —
    // it reads the `AppExit` that `run()` returns instead, and makes its code
    // the process's.
    exit.write(verdict.exit());
}
