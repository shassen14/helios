pub mod finalize;
pub mod resolve;
pub mod setup;
pub mod state;
pub mod termination_check;
pub mod tick;

#[cfg(test)]
mod fixtures;

pub use tick::TickAction;

use crate::{
    assertion::{extract::standard_extractors, Assertion},
    collector::ObservationCollector,
    run::{
        validation::{self, ValidationError},
        Run,
    },
    runner::resolve::ResolvedPlan,
};

use helios_core::prelude::MonotonicTime;

/// Why a run aborts when the host ticks or finishes it without having set
/// it up.
const NEVER_SET_UP: &str = "the host never called on_pipeline_built";

/// Why a run aborts when the host hands it a collector it wasn't set up
/// against.
const FOREIGN_COLLECTOR: &str = "the collector is not the one the run was set up against";

/// _Per-step._ The state machine the host pumps across a run: owns the run,
/// its phase (which holds the per-assertion state once the run is ready), and
/// its clock: the first tick's time and the latest's.
pub struct Runner {
    run: Run,
    phase: Phase,
    start_time: Option<MonotonicTime>,
    latest_tick: Option<MonotonicTime>,
    setup_failures: Vec<String>,
}

impl Runner {
    /// Build a runner from a parsed run, rejecting an incoherent one up front.
    /// Validation runs here so a bad run fails at *construction*, never partway
    /// through a simulation. It can't tick until `on_pipeline_built`, the one
    /// moment the pipelines exist to check its assertions against.
    pub fn new(run: Run) -> Result<Self, ValidationError> {
        // Coherence check first: `?` bails with the validation error, so an
        // invalid run never gets a half-built runner.
        validation::validate(&run)?;

        Ok(Self {
            run,
            phase: Phase::AwaitingPipelines,
            start_time: None,
            latest_tick: None,
            // Filled only if the run is doomed before it ticks. Assertions
            // cannot speak for this class of failure: a scenario whose agent
            // has no pipeline usually has no assertion naming it either, so
            // an assertion-only verdict reports a vacuous pass.
            setup_failures: Vec::new(),
        })
    }

    /// The collector this run reads: one requesting every target its
    /// assertions name. A host builds it here, beside the runner, so the
    /// two always describe the same run.
    pub fn collector(&self) -> ObservationCollector {
        ObservationCollector::new(
            self.run.assertions().iter().map(Assertion::target),
            standard_extractors(),
        )
    }

    /// Record that the run is unfit to produce a meaningful result, and why.
    ///
    /// Called before the first tick, by the host, for conditions no assertion
    /// can observe. Any recorded failure forces the run's verdict to
    /// [`ReportStatus::Failed`] in [`Self::finalize`], independent of how the
    /// assertions land — including when there are none. From then on the
    /// run judges nothing, whatever phase it was in.
    pub fn fail_setup(&mut self, reason: String) {
        self.setup_failures.push(reason);
        self.phase = Phase::Aborted;
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
    /// Setup failed; nothing is judged.
    Aborted,
}

#[cfg(test)]
mod tests {
    use super::*;

    // Parse a `Run` from TOML, filling in the header fields every run needs so
    // each test supplies only its termination + assertion body.
    fn run_with(body: &str) -> Run {
        let toml = format!(
            r#"
            schema_version = "1.0"
            scenario = {{ from = "s" }}
            observation = {{ from = "o" }}
            {body}
            "#
        );
        toml::from_str(&toml).expect("valid run toml")
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

    #[test]
    fn a_setup_failure_aborts_the_run() {
        let mut r = Runner::new(run_with("termination = { max_simulated_seconds = 5.0 }"))
            .expect("valid run");
        r.fail_setup("no pipeline".to_string());
        assert_eq!(r.phase, Phase::Aborted);
    }
}
