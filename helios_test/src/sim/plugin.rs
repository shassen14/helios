use super::{ActiveRunner, RunOutcome, RunVerdict, WallClockStart};
use crate::run::termination::TerminationReason;
use crate::sim::{ReportOutputPath, RunMetadata};
use crate::{AgentId, ReportStatus, TickAction};

use helios_core::data::MonotonicTime;
use helios_runtime::port::PortBus;
use helios_runtime::AutonomyPipeline;
use helios_sim::prelude::{AppState, SimulationSet};
use helios_sim::simulation::plugins::autonomy::components::{
    AgentIdComponent, AutonomyPipelineComponent,
};

use bevy::prelude::*;

pub struct TestRunnerPlugin;

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
/// pipelines so it can resolve assertion targets to bus channels.
///
/// An `Err` here means an assertion names a target no agent produces — a
/// legitimate **test failure** (exit 1), not a crash: mark the verdict failed
/// and route to `Flushing` so it flows through the normal report path.
fn on_pipeline_built_system(
    query: Query<(&AgentIdComponent, &AutonomyPipelineComponent)>,
    mut runner: ResMut<ActiveRunner>,
    mut next_state: ResMut<NextState<AppState>>,
    mut run_verdict: ResMut<RunVerdict>,
) {
    // The query returns only agents with BOTH components — guaranteed by the
    // co-located insert in spawn_autonomy_pipeline (see AgentIdComponent docs).
    let mut pairs: Vec<(AgentId, &AutonomyPipeline)> = Vec::new();
    for (agent_id_comp, autonomy_pipeline_comp) in query.iter() {
        pairs.push((
            AgentId::new(agent_id_comp.0.clone()),
            &autonomy_pipeline_comp.0,
        ));
    }

    match runner.0.on_pipeline_built(&pairs) {
        Ok(()) => (),
        Err(e) => {
            tracing::error!("on_pipeline_built failed: {e}");
            run_verdict.set(ReportStatus::Failed);
            next_state.set(AppState::Flushing);
        }
    }
}

/// System 2 — runs every `FixedUpdate` in `SimulationSet::Validation`, which is
/// ordered after `Estimation`/`StateSync`, so the brain's outputs and the oracle
/// channels are already fresh when the runner observes the buses.
///
/// The runner owns termination (via the run file's `termination` block); this
/// system only honors the `Terminate` it is handed by stashing the reason and
/// routing to `Flushing`. No host-side timeout lives here.
fn tick_system(
    query: Query<(&AgentIdComponent, &AutonomyPipelineComponent)>,
    time: Res<Time>,
    mut runner: ResMut<ActiveRunner>,
    mut outcome: ResMut<RunOutcome>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    // Same clock source as the brain's tick, so the time budget and the brain
    // never disagree.
    let now = MonotonicTime(time.elapsed_secs_f64());

    let mut pairs: Vec<(AgentId, &PortBus)> = Vec::new();

    for (agent_id_comp, autonomy_pipeline_comp) in query.iter() {
        pairs.push((
            AgentId::new(agent_id_comp.0.clone()),
            autonomy_pipeline_comp.0.bus(),
        ));
    }

    match runner.0.tick(now, &pairs) {
        TickAction::Terminate { reason } => {
            outcome.reason = Some(reason);
            next_state.set(AppState::Flushing);
        }
        TickAction::Continue => (),
    }
}

/// System 3 — runs once at `OnEnter(AppState::Flushing)`. Builds the final
/// `Report` (terminal assertions judged now; continuous ones already folded
/// across ticks into their latches), writes it to disk and the console, records
/// the verdict, and ends the Bevy loop. Takes the runner as `Res` because
/// `finalize` only reads.
// Bevy systems declare their dependencies as parameters, so the count exceeds
// clippy's default threshold; that's idiomatic here, not a smell.
#[allow(clippy::too_many_arguments)]
fn finalize_system(
    query: Query<(&AgentIdComponent, &AutonomyPipelineComponent)>,
    time: Res<Time>,
    wall_start: Res<WallClockStart>,
    runner: Res<ActiveRunner>,
    outcome: Res<RunOutcome>,
    out_path: Res<ReportOutputPath>,
    metadata: Res<RunMetadata>,
    mut verdict: ResMut<RunVerdict>,
    mut exit: EventWriter<AppExit>,
) {
    let now = MonotonicTime(time.elapsed_secs_f64());
    let wall_secs = wall_start.0.elapsed().as_secs_f64();
    let run_name = outcome.run_name.clone();
    // `None` only on the "terminated before any tick" path (system 1's
    // short-circuit): setup never produced a tick, so there is no real
    // termination cause. `Aborted` names that case instead of borrowing a
    // misleading stand-in like the time budget.
    let reason = outcome.reason.clone().unwrap_or(TerminationReason::Aborted);

    // TODO(perf): SmallVec / scratch buffer at swarm scale.
    let mut pairs: Vec<(AgentId, &PortBus)> = Vec::new();
    for (agent_id_comp, autonomy_pipeline_comp) in query.iter() {
        pairs.push((
            AgentId::new(agent_id_comp.0.clone()),
            autonomy_pipeline_comp.0.bus(),
        ));
    }

    let report = runner
        .0
        .finalize(now, wall_secs, run_name, reason, &pairs, metadata.seed);

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
