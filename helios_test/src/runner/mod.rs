pub mod finalize;
pub mod resolve;
pub mod setup;
pub mod state;
pub mod termination_check;
pub mod tick;
pub mod window;

#[cfg(test)]
mod fixtures;

pub use finalize::HostFacts;
pub use setup::UnbuiltPipeline;
pub use tick::TickAction;

use crate::{
    assertion::extract::standard_extractors,
    collector::ObservationCollector,
    evaluation::{standard_evaluators, EvaluatorRegistry},
    report::{AbortCause, HostFault},
    run::{
        validation::{self, ValidationError},
        Run,
    },
    runner::resolve::{evaluations::PlannedEvaluation, ResolvedPlan, Unusable},
};

use helios_core::prelude::MonotonicTime;

/// _Per-step._ The state machine the host pumps across a run: owns the run,
/// its phase (which holds each entry's state once the run is ready), and
/// its clock: the span its ticks have covered.
pub struct Runner {
    run: Run,
    /// The kinds its evaluations name, built from at setup.
    evaluators: EvaluatorRegistry,
    phase: Phase,
    /// `None` until the first tick.
    ticks: Option<TickSpan>,
}

impl Runner {
    /// Build a runner from a parsed run, rejecting an incoherent one up front.
    /// Validation runs here so a bad run fails at *construction*, never partway
    /// through a simulation. It can't tick until `on_pipeline_built`, the one
    /// moment the pipelines exist to check its assertions against.
    pub fn new(run: Run) -> Result<Self, ValidationError> {
        Self::with_evaluators(run, standard_evaluators())
    }

    /// As [`new`](Self::new), its evaluations built from the kinds in
    /// `evaluators` rather than the standard ones.
    pub fn with_evaluators(
        run: Run,
        evaluators: EvaluatorRegistry,
    ) -> Result<Self, ValidationError> {
        // Coherence check first: `?` bails with the validation error, so an
        // invalid run never gets a half-built runner.
        validation::validate(&run, &evaluators)?;

        Ok(Self {
            run,
            evaluators,
            phase: Phase::AwaitingPipelines,
            ticks: None,
        })
    }

    /// The collector this run reads. It requests nothing until setup binds
    /// what the run's entries and evaluations read, so a host asks it what
    /// to record only after [`on_pipeline_built`](Self::on_pipeline_built).
    /// A host builds it here, beside the runner, so the two always describe
    /// the same run.
    pub fn collector(&self) -> ObservationCollector {
        ObservationCollector::new(standard_extractors())
    }

    /// Whether the run was aborted untested: setup refused it, or the host
    /// broke its contract with the runner. An aborted run judges nothing, so
    /// a host stops it and finalizes, and the report says why.
    pub fn is_aborted(&self) -> bool {
        matches!(self.phase, Phase::Aborted(_))
    }
}

/// _Per-step._ Where the run is in its life, so each step can tell whether
/// it may judge.
// An enum field rather than a type parameter (`Runner<Ready>`): the runner is
// held in a Bevy resource, which can't change type in place.
#[derive(Debug, Clone, PartialEq)]
enum Phase {
    /// Built, but not yet checked against the pipelines.
    AwaitingPipelines,
    /// Every assertion can be judged, from the series the plan names, and
    /// the plan holds each one's state.
    Ready(ResolvedPlan),
    /// Stopped untested, for what the abort holds; nothing is judged.
    Aborted(Abort),
}

/// _Per-step._ Why the runner stopped a run untested, as far as it found
/// before it stopped looking: setup checks every agent and entry at once, and
/// a host fault ends the run where it is found.
#[derive(Debug, Clone, PartialEq, Default)]
struct Abort {
    /// Each agent whose pipeline the host couldn't build.
    unbuilt: Vec<UnbuiltPipeline>,
    /// The evaluations and entries setup refused.
    unusable: Unusable,
    /// The evaluations setup bound, which the run stopped before deriving.
    bound: Vec<PlannedEvaluation>,
    fault: Option<HostFault>,
}

impl Abort {
    /// An abort for the host's `fault` alone.
    fn fault(fault: HostFault) -> Self {
        Self {
            fault: Some(fault),
            ..Self::default()
        }
    }

    /// An abort for the host's `fault`, found once setup had bound `plan`.
    fn fault_in(fault: HostFault, plan: &ResolvedPlan) -> Self {
        Self {
            bound: plan.evaluations().to_vec(),
            ..Self::fault(fault)
        }
    }

    /// Each cause the abort holds, as the report names it: the pipelines not
    /// built, then the host's fault. Whether entries were refused the report
    /// derives from the entries themselves.
    fn causes(&self) -> Vec<AbortCause> {
        self.unbuilt
            .iter()
            .map(|unbuilt| AbortCause::PipelineNotBuilt {
                agent: unbuilt.agent.as_str().to_owned(),
                errors: unbuilt.errors.clone(),
            })
            .chain(self.fault.clone().map(AbortCause::HostFault))
            .collect()
    }
}

/// _Per-step._ The simulated time a run's ticks have covered: its first
/// tick, where the run's clock starts (`run.start`), and its latest
/// (`run.end`, once the run stops).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TickSpan {
    pub first: MonotonicTime,
    pub latest: MonotonicTime,
}

impl TickSpan {
    /// The span of one tick at `t`.
    fn starting_at(t: MonotonicTime) -> Self {
        Self {
            first: t,
            latest: t,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Parse a `Run` from TOML, filling in the header fields every run needs so
    // each test supplies only its termination + assertion body.
    fn run_with(body: &str) -> Run {
        let toml = format!(
            r#"
            schema_version = "2.0"
            scenario = {{ from = "s" }}
            observation = {{ from = "o" }}
            {body}
            "#
        );
        crate::run::parse(&toml, crate::run::no_suite_files).expect("valid run toml")
    }

    #[test]
    fn rejects_an_incoherent_run() {
        // A NaN budget is never reached; `new` must surface the validation
        // error rather than build a runner that can never terminate. `matches!`
        // (not `unwrap_err`) avoids needing `Debug` on the `Ok` type `Runner`.
        let run = run_with("termination = { max_simulated_seconds = nan }");
        assert!(matches!(
            Runner::new(run),
            Err(ValidationError::UnreachableBudget(_))
        ));
    }

    #[test]
    fn a_new_runner_awaits_its_pipelines() {
        let r = Runner::new(run_with("termination = { max_simulated_seconds = 5.0 }"))
            .expect("valid run");
        assert_eq!(r.phase, Phase::AwaitingPipelines);
    }
}
