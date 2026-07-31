pub mod lifecycle;
pub mod state;
pub mod termination_check;
pub mod tick;

pub use tick::TickAction;

use crate::{
    assertion::{
        extract::{standard_extractors, ExtractorTable},
        target::TargetRegistry,
    },
    run::{
        validation::{self, ValidationError},
        Run,
    },
    runner::state::AssertionState,
};

use helios_core::data::MonotonicTime;

/// _Per-step._ The state machine the host pumps across a run: owns the run, the
/// resolved registry, the extractor table, per-assertion state, and the clock
/// origin.
pub struct Runner {
    run: Run,
    registry: TargetRegistry,
    extractors: ExtractorTable,
    states: Vec<AssertionState>,
    start_time: Option<MonotonicTime>,
    setup_failures: Vec<String>,
}

impl Runner {
    /// Build a runner from a parsed run, rejecting an incoherent one up front.
    /// Validation runs here so a bad run fails at *construction*, never partway
    /// through a simulation. The registry starts empty — it's populated later in
    /// `on_pipeline_built`, the one moment the pipeline exists to walk.
    pub fn new(run: Run) -> Result<Self, ValidationError> {
        // Coherence check first: `?` bails with the validation error before any
        // state is allocated, so an invalid run never gets a half-built runner.
        validation::validate(&run)?;

        // One fresh latch per assertion, index-aligned with `run.assertions()`
        // so the runner addresses an assertion and its state by the same index.
        // Built from a borrow that ends here, before `run` is moved into Self.
        let states: Vec<AssertionState> = run
            .assertions()
            .iter()
            .map(|_| AssertionState::new())
            .collect();

        Ok(Self {
            run,
            // Empty until `on_pipeline_built` walks each pipeline and registers
            // its declared channels.
            registry: TargetRegistry::new(),
            // The standard f64/bool/i64 extractors. An empty table here would
            // make every assertion fail with `NoExtractor`, since the evaluator
            // couldn't turn any payload into a comparable value.
            extractors: standard_extractors(),
            states,
            start_time: None,
            // Filled only if the run is doomed before it ticks. Assertions
            // cannot speak for this class of failure: a scenario whose agent
            // has no pipeline usually has no assertion naming it either, so
            // an assertion-only verdict reports a vacuous pass.
            setup_failures: Vec::new(),
        })
    }

    /// Record that the run is unfit to produce a meaningful result, and why.
    ///
    /// Called before the first tick, by the host, for conditions no assertion
    /// can observe. Any recorded failure forces the run's verdict to
    /// [`ReportStatus::Failed`] in [`Self::finalize`], independent of how the
    /// assertions land — including when there are none.
    pub fn fail_setup(&mut self, reason: String) {
        self.setup_failures.push(reason);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::runner::state::ContinuousStatus;

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

    const ASSERT: &str = r#"
        [[assert]]
        when = "continuous"
        target = "agent.car.x"
        condition = "equals"
        value = 0.0
    "#;

    #[test]
    fn rejects_an_incoherent_run() {
        // Empty termination never stops; `new` must surface the validation
        // error rather than build a runner that can never terminate. `matches!`
        // (not `unwrap_err`) avoids needing `Debug` on the `Ok` type `Runner`.
        let run = run_with("termination = {}");
        assert!(matches!(
            Runner::new(run),
            Err(ValidationError::NoTermination)
        ));
    }

    #[test]
    fn allocates_one_state_per_assertion() {
        // States are index-aligned with the assertions, so the count must match.
        let body = format!(r#"termination = {{ max_simulated_seconds = 5.0 }}{ASSERT}{ASSERT}"#);
        let r = Runner::new(run_with(&body)).expect("valid run");
        assert_eq!(r.states.len(), 2);
    }

    #[test]
    fn fresh_states_start_pending() {
        let body = format!(r#"termination = {{ max_simulated_seconds = 5.0 }}{ASSERT}"#);
        let r = Runner::new(run_with(&body)).expect("valid run");
        assert_eq!(r.states[0].status(), &ContinuousStatus::Pending);
    }

    #[test]
    fn a_smoke_run_has_no_states() {
        // Zero assertions is valid when a time budget guarantees termination.
        let r = Runner::new(run_with("termination = { max_simulated_seconds = 5.0 }"))
            .expect("valid run");
        assert!(r.states.is_empty());
    }
}
