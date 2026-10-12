use super::{
    resolve::{join, resolve, UnusableAssertion},
    Runner,
};
use crate::{
    assertion::{
        evaluator::evaluate, target::build_for_pipeline, Assertion, AssertionResult, FailureKind,
    },
    collector::ObservationCollector,
    report::{AssertionReportEntry, AssertionStatus, Report, ReportStatus, TerminatedBy},
    run::termination::TerminationReason,
};
use helios_core::prelude::{AgentId, MonotonicTime};
use helios_runtime::{port::PortBus, AutonomyPipeline};

use thiserror::Error;

impl Runner {
    /// Check, before the first tick, that the run can judge every assertion
    /// against what `pipelines` report and what `collector` can extract.
    pub fn on_pipeline_built(
        &mut self,
        pipelines: &[(AgentId, &AutonomyPipeline)],
        collector: &ObservationCollector,
    ) -> Result<(), LifecycleError> {
        // `tick` still reads each target's channel off the bus, through this
        // registry.
        for (agent, pipeline) in pipelines {
            for (target, channel) in build_for_pipeline(agent, pipeline) {
                self.registry.register(agent.clone(), target, channel);
            }
        }

        let unusable = resolve(self.run.assertions(), pipelines, collector);
        if unusable.is_empty() {
            Ok(())
        } else {
            Err(LifecycleError::UnusableAssertions(unusable))
        }
    }

    /// Build the run's [`Report`] once the host stops ticking. Continuous
    /// assertions are read from their latches (already folded across every
    /// tick by `tick`); terminal assertions are judged here, once, against the
    /// final bus. The run is `Passed` only if every assertion passed — any
    /// `Failed`, or a never-observed `Pending`, drops it to `Failed`.
    ///
    /// Takes `&self`, not `&mut`: the latches are only read, nothing is
    /// recorded. `run_name`, `wall_duration_secs`, the stop `reason`, and
    /// `master_seed` are supplied by the host — the runner reads no wall clock,
    /// stores no name, and does not own the seed. `master_seed` is `None` for an
    /// unseeded run, whose real seed is chosen inside the sim and unknown here.
    pub fn finalize(
        &self,
        sim_now: MonotonicTime,
        wall_duration_secs: f64,
        run_name: String,
        terminated_by_reason: TerminationReason,
        blackboards: &[(AgentId, &PortBus)],
        master_seed: Option<u64>,
    ) -> Report {
        let mut entries = Vec::with_capacity(self.run.assertions().len());
        // Run-level verdict accumulator: flipped false the moment any assertion
        // is not Passed, so it survives the loop without a second pass.
        let mut all_passed = true;

        for (i, assertion) in self.run.assertions().iter().enumerate() {
            // Each branch produces the two things the entry needs — a verdict
            // and the observed value — straight from the data it already holds.
            // Continuous reads its latch (its `last_value` is already an
            // `Option`); terminal is judged now and the value pulled from the
            // fresh result.
            let (status, actual) = match assertion {
                Assertion::Continuous(_) => (
                    AssertionStatus::from(self.states[i].status()),
                    self.states[i].last_value().clone(),
                ),
                Assertion::Terminal(_) => {
                    let result = self.result_for(assertion, blackboards);
                    // `from(&result)` borrows for the verdict; `into_actual`
                    // then moves the value out — borrow ends before the move,
                    // so this tuple order is what keeps both legal.
                    (AssertionStatus::from(&result), result.into_actual())
                }
            };

            if !matches!(status, AssertionStatus::Passed) {
                all_passed = false;
            }

            entries.push(AssertionReportEntry::new(
                assertion.target().as_str().to_string(),
                *assertion.condition(),
                assertion.expected().clone(),
                actual,
                status,
            ));
        }

        // A setup failure outranks the assertion tally: a run whose scene never
        // became fit to test cannot pass, however few assertions were left to
        // judge it.
        let run_status = if all_passed && self.setup_failures.is_empty() {
            ReportStatus::Passed
        } else {
            ReportStatus::Failed
        };

        // Elapsed simulated time from the origin `tick` stamped on its first
        // call. `None` only if finalize is reached with no tick having run;
        // report zero rather than panic.
        let simulated_duration_secs = match self.start_time {
            Some(start) => sim_now.0 - start.0,
            None => 0.0f64,
        };

        Report::new(
            run_name,
            self.run.scenario().path().to_string(),
            master_seed,
            run_status,
            self.setup_failures.clone(),
            simulated_duration_secs,
            wall_duration_secs,
            TerminatedBy::from(&terminated_by_reason),
            entries,
        )
    }

    /// Judge one assertion against the current bus state — the resolve →
    /// find-bus → evaluate path shared by `tick`'s per-tick pass and
    /// `finalize`'s terminal pass, so the two can't drift. A target that
    /// resolved at setup but whose bus is absent now degrades to an
    /// `UnresolvedTarget` failure rather than panicking mid-run.
    pub(super) fn result_for(
        &self,
        assertion: &Assertion,
        blackboards: &[(AgentId, &PortBus)],
    ) -> AssertionResult {
        // Find the agent whose registry maps this target to a channel. The
        // closure receives `&&(AgentId, &PortBus)`; match ergonomics bind
        // `agent` as `&AgentId`, which is what `resolve` wants.
        let resolved = blackboards
            .iter()
            .find(|(agent, _)| self.registry.resolve(agent, assertion.target()).is_some());

        match resolved {
            // `bus` is `&&PortBus` here; deref coercion at the call site
            // hands `evaluate` the `&PortBus` it expects.
            Some((agent, bus)) => evaluate(assertion, agent, &self.registry, bus, &self.extractors),
            // `on_pipeline_built` already proved every target resolves, so
            // this only fires if a bus went missing between setup and now.
            // Degrade to a recorded failure rather than panic mid-run.
            None => AssertionResult::Failed {
                kind: FailureKind::UnresolvedTarget,
                actual: None,
            },
        }
    }
}

/// Between the unusable assertions in one message. An assertion's own
/// message holds a `,` but never this.
const ASSERTION_SEPARATOR: &str = "; ";

#[derive(Debug, Clone, PartialEq, Error)]
pub enum LifecycleError {
    /// Assertions the run can't judge, each with its reasons, so the
    /// operator sees every one to fix at once.
    #[error(
        "the run can't judge these assertions: {}",
        join(.0, ASSERTION_SEPARATOR)
    )]
    UnusableAssertions(Vec<UnusableAssertion>),
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::extract::{standard_extractors, ExtractorTable};
    use crate::assertion::target::TargetRegistry;
    use crate::assertion::AssertionTarget;
    use crate::run::Run;
    use crate::runner::resolve::TargetProblem;

    use helios_runtime::PipelineBuilder;

    // A node-less pipeline: its agent reports nothing, so every target on it
    // is unresolved. What resolves is tested beside `resolve`.
    fn empty_pipeline() -> AutonomyPipeline {
        PipelineBuilder::new()
            .build()
            .expect("an empty pipeline builds without errors")
    }

    // Assemble a `Runner` directly, with an empty registry and no extractors:
    // these tests judge nothing.
    fn runner(run: Run, registry: TargetRegistry) -> Runner {
        Runner {
            run,
            registry,
            extractors: ExtractorTable::new(),
            states: Vec::new(),
            start_time: None,
            setup_failures: Vec::new(),
        }
    }

    // Build a `Run` (via TOML, since its fields are private) carrying one
    // terminal assertion per target string. Termination is filled so the run is
    // structurally complete; only the assertion targets matter here.
    fn run_with_targets(targets: &[&str]) -> Run {
        let mut text = String::from(
            "schema_version = \"1.0\"\n\
             scenario = { from = \"s\" }\n\
             observation = { from = \"o\" }\n\
             termination = { max_simulated_seconds = 5.0 }\n",
        );
        for t in targets {
            text.push_str(&format!(
                "\n[[assert]]\n\
                 when = \"at_completion\"\n\
                 target = \"{t}\"\n\
                 condition = \"equals\"\n\
                 value = 0.0\n"
            ));
        }
        toml::from_str(&text).expect("valid run toml")
    }

    // The collector the host would build for `r`'s run.
    fn collector(r: &Runner) -> ObservationCollector {
        ObservationCollector::new(
            r.run.assertions().iter().map(Assertion::target),
            standard_extractors(),
        )
    }

    fn target(path: &str) -> AssertionTarget {
        AssertionTarget::parse(path).expect("valid target")
    }

    #[test]
    fn zero_assertions_is_ok() {
        // A pure smoke run: nothing to resolve, so nothing can fail.
        let mut r = runner(run_with_targets(&[]), TargetRegistry::new());
        let collector = collector(&r);
        assert_eq!(r.on_pipeline_built(&[], &collector), Ok(()));
    }

    #[test]
    fn every_unusable_target_is_rejected_at_once() {
        // Caught here, before anything runs, and all reported together:
        // fixing one only to meet the next is a slow loop.
        let agent = AgentId::new("car");
        let pipeline = empty_pipeline();
        let mut r = runner(
            run_with_targets(&["agent.car.estimator.a", "agent.car.estimator.b"]),
            TargetRegistry::new(),
        );
        let collector = collector(&r);
        assert_eq!(
            r.on_pipeline_built(&[(agent, &pipeline)], &collector),
            Err(LifecycleError::UnusableAssertions(vec![
                UnusableAssertion {
                    number: 1,
                    target: target("agent.car.estimator.a"),
                    problems: vec![TargetProblem::Unreported],
                },
                UnusableAssertion {
                    number: 2,
                    target: target("agent.car.estimator.b"),
                    problems: vec![TargetProblem::Unreported],
                },
            ]))
        );
    }

    // Build the report for a run that never ticked, the way `finalize_system`
    // does on the abort path.
    fn finalize_aborted(r: &Runner) -> crate::report::Report {
        r.finalize(
            MonotonicTime(0.0),
            0.0,
            "run".to_string(),
            TerminationReason::Aborted,
            &[],
            None,
        )
    }

    #[test]
    fn a_setup_failure_fails_a_run_with_no_assertions() {
        // The regression this guards: a scenario whose agent has no pipeline
        // usually has no assertion naming it either. Judged on assertions
        // alone, an empty set is a vacuous pass and the run reports success
        // for a scene that could never have been tested.
        let mut r = runner(run_with_targets(&[]), TargetRegistry::new());
        r.fail_setup("agent 'car' has no autonomy pipeline".to_string());

        let report = finalize_aborted(&r);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert!(report.assertions().is_empty());
    }

    #[test]
    fn a_setup_failure_is_reported_with_its_reason() {
        // status alone tells CI a run failed; without the reason there is
        // nothing in the report to say why, since no assertion failed.
        let mut r = runner(run_with_targets(&[]), TargetRegistry::new());
        r.fail_setup("first".to_string());
        r.fail_setup("second".to_string());

        let report = finalize_aborted(&r);

        assert_eq!(report.setup_failures(), ["first", "second"]);
    }

    #[test]
    fn a_clean_run_records_no_setup_failures() {
        let r = runner(run_with_targets(&[]), TargetRegistry::new());

        let report = finalize_aborted(&r);

        assert_eq!(report.status(), &ReportStatus::Passed);
        assert!(report.setup_failures().is_empty());
    }
}
