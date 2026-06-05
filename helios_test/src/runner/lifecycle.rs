use super::Runner;
use crate::{
    assertion::{
        evaluator::evaluate,
        target::{build_for_pipeline, AgentId},
        Assertion, AssertionResult, AssertionTarget, FailureKind,
    },
    report::{AssertionReportEntry, AssertionStatus, Report, ReportStatus, TerminatedBy},
    run::termination::TerminationReason,
};
use helios_core::data::MonotonicTime;
use helios_runtime::{port::PortBus, AutonomyPipeline};

use thiserror::Error;

impl Runner {
    pub fn on_pipeline_built(
        &mut self,
        pipelines: &[(AgentId, &AutonomyPipeline)],
    ) -> Result<(), LifecycleError> {
        for (agent, pipeline) in pipelines {
            for (target, channel) in build_for_pipeline(agent, pipeline) {
                self.registry.register(agent.clone(), target, channel);
            }
        }

        let mut failures: Vec<AssertionTarget> = Vec::new();

        for assertion in self.run.assertions() {
            let run_target = assertion.target();

            // Essentially do any of the agents produce a target value via
            // channel. If they don't, we push target to failure since we
            // cannot test against this assertion value
            if !pipelines
                .iter()
                .any(|(agent, _)| self.registry.resolve(agent, run_target).is_some())
            {
                failures.push(run_target.clone());
            }
        }

        if failures.is_empty() {
            Ok(())
        } else {
            Err(LifecycleError::UnresolvedTargets(failures))
        }
    }

    /// Build the run's [`Report`] once the host stops ticking. Continuous
    /// assertions are read from their latches (already folded across every
    /// tick by `tick`); terminal assertions are judged here, once, against the
    /// final bus. The run is `Passed` only if every assertion passed — any
    /// `Failed`, or a never-observed `Pending`, drops it to `Failed`.
    ///
    /// Takes `&self`, not `&mut`: the latches are only read, nothing is
    /// recorded. `run_name`, `wall_duration_secs`, and the stop `reason` are
    /// supplied by the host — the runner reads no wall clock and stores no name
    /// — and `master_seed` is `None` until the determinism feature lands.
    pub fn finalize(
        &self,
        sim_now: MonotonicTime,
        wall_duration_secs: f64,
        run_name: String,
        terminated_by_reason: TerminationReason,
        blackboards: &[(AgentId, &PortBus)],
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

        let run_status = if all_passed {
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
            None, // todo: input when seed number matters for determinism
            run_status,
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

#[derive(Debug, Clone, PartialEq, Error)]
pub enum LifecycleError {
    // `{0:?}` surfaces the offending targets in the message — the whole point
    // of carrying them is so the operator sees *which* ones to fix.
    #[error("unresolved assertion targets: {0:?}")]
    UnresolvedTargets(Vec<AssertionTarget>),
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::extract::ExtractorTable;
    use crate::assertion::target::TargetRegistry;
    use crate::run::Run;

    use helios_runtime::port::InternalChannel;
    use helios_runtime::PipelineBuilder;

    // A channel-less pipeline. `build_for_pipeline` yields nothing for it, so a
    // test controls exactly what the registry contains by pre-seeding instead.
    fn empty_pipeline() -> AutonomyPipeline {
        PipelineBuilder::new()
            .build()
            .expect("an empty pipeline builds without errors")
    }

    // Assemble a `Runner` directly. `new()` isn't implemented yet, and these
    // tests exercise only `on_pipeline_built`, which touches `run` + `registry`.
    fn runner(run: Run, registry: TargetRegistry) -> Runner {
        Runner {
            run,
            registry,
            extractors: ExtractorTable::new(),
            states: Vec::new(),
            start_time: None,
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

    // Register `target` for `agent` as if the pipeline had declared its channel.
    fn seed(registry: &mut TargetRegistry, agent: &AgentId, target: &str) {
        registry.register(
            agent.clone(),
            AssertionTarget::new(target),
            InternalChannel::of::<f64>().into(),
        );
    }

    #[test]
    fn zero_assertions_is_ok() {
        // A pure smoke run: nothing to resolve, so nothing can fail.
        let mut r = runner(run_with_targets(&[]), TargetRegistry::new());
        assert_eq!(r.on_pipeline_built(&[]), Ok(()));
    }

    #[test]
    fn unresolved_target_is_rejected() {
        // The pipeline declares no channels, so the assertion's target maps to
        // nothing — caught here, before anything runs.
        let agent = AgentId::new("car");
        let pipeline = empty_pipeline();
        let mut r = runner(
            run_with_targets(&["agent.car.missing"]),
            TargetRegistry::new(),
        );
        assert_eq!(
            r.on_pipeline_built(&[(agent, &pipeline)]),
            Err(LifecycleError::UnresolvedTargets(vec![
                AssertionTarget::new("agent.car.missing")
            ]))
        );
    }

    #[test]
    fn all_unresolved_targets_are_collected() {
        // Reporting every failure at once beats failing on the first.
        let agent = AgentId::new("car");
        let pipeline = empty_pipeline();
        let mut r = runner(
            run_with_targets(&["agent.car.a", "agent.car.b"]),
            TargetRegistry::new(),
        );
        assert_eq!(
            r.on_pipeline_built(&[(agent, &pipeline)]),
            Err(LifecycleError::UnresolvedTargets(vec![
                AssertionTarget::new("agent.car.a"),
                AssertionTarget::new("agent.car.b"),
            ]))
        );
    }

    #[test]
    fn resolved_target_passes() {
        let agent = AgentId::new("car");
        let pipeline = empty_pipeline();
        let mut registry = TargetRegistry::new();
        seed(&mut registry, &agent, "agent.car.x");

        let mut r = runner(run_with_targets(&["agent.car.x"]), registry);
        assert_eq!(r.on_pipeline_built(&[(agent, &pipeline)]), Ok(()));
    }

    #[test]
    fn reports_only_the_unresolved_in_a_mix() {
        // One target resolves, one doesn't; only the failure is reported.
        let agent = AgentId::new("car");
        let pipeline = empty_pipeline();
        let mut registry = TargetRegistry::new();
        seed(&mut registry, &agent, "agent.car.good");

        let mut r = runner(
            run_with_targets(&["agent.car.good", "agent.car.bad"]),
            registry,
        );
        assert_eq!(
            r.on_pipeline_built(&[(agent, &pipeline)]),
            Err(LifecycleError::UnresolvedTargets(vec![
                AssertionTarget::new("agent.car.bad")
            ]))
        );
    }
}
