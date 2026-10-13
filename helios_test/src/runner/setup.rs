use super::{
    resolve::{join, resolve, UnusableAssertion},
    Phase, Runner,
};
use crate::collector::ObservationCollector;
use helios_core::prelude::AgentId;
use helios_runtime::AutonomyPipeline;

use thiserror::Error;

impl Runner {
    /// Check, before the first tick, that the run can judge every assertion
    /// against what `pipelines` report and what `collector` records, and keep
    /// what the check found for every tick. The run is then ready, or, if
    /// any assertion can't be judged, aborted with the reasons.
    ///
    /// Called once. A second call, or one after setup failed, is refused and
    /// changes nothing.
    pub fn on_pipeline_built(
        &mut self,
        pipelines: &[(AgentId, &AutonomyPipeline)],
        collector: &ObservationCollector,
    ) -> Result<(), SetupError> {
        if self.phase != Phase::AwaitingPipelines {
            return Err(SetupError::AlreadySetUp);
        }

        match resolve(self.run.assertions(), pipelines, collector) {
            Ok(plan) => {
                self.phase = Phase::Ready(plan);
                Ok(())
            }
            Err(unusable) => {
                self.phase = Phase::Aborted;
                Err(SetupError::UnusableAssertions(unusable))
            }
        }
    }
}

/// Between the unusable assertions in one message. An assertion's own
/// message holds a `,` but never this.
const ASSERTION_SEPARATOR: &str = "; ";

#[derive(Debug, Clone, PartialEq, Error)]
pub enum SetupError {
    /// Assertions the run can't judge, each with its reasons, so the
    /// operator sees every one to fix at once.
    #[error(
        "the run can't judge these assertions: {}",
        join(.0, ASSERTION_SEPARATOR)
    )]
    UnusableAssertions(Vec<UnusableAssertion>),
    /// `on_pipeline_built` was called after the run's setup was decided.
    #[error("the run's setup is already decided")]
    AlreadySetUp,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::AssertionTarget,
        runner::{
            fixtures::{pipeline, ready, run, term, CAR},
            resolve::TargetProblem,
        },
    };

    use helios_runtime::PipelineBuilder;

    // A node-less pipeline: its agent reports nothing, so every target on it
    // is unreported. What resolves is tested beside `resolve`.
    fn empty_pipeline() -> AutonomyPipeline {
        PipelineBuilder::new()
            .build()
            .expect("an empty pipeline builds without errors")
    }

    fn target(path: &str) -> AssertionTarget {
        AssertionTarget::parse(path).expect("valid target")
    }

    #[test]
    fn zero_assertions_is_ok() {
        // A pure smoke run: nothing to resolve, so nothing can fail.
        let mut r = Runner::new(run("max_simulated_seconds = 5.0", &[])).expect("valid run");
        let collector = r.collector();
        assert_eq!(r.on_pipeline_built(&[], &collector), Ok(()));
    }

    #[test]
    fn a_run_whose_assertions_can_all_be_judged_is_ready() {
        let (r, _) = ready(
            run("max_simulated_seconds = 5.0", &[&term(CAR, 5.0)]),
            &[CAR],
        );
        assert!(matches!(r.phase, Phase::Ready(_)));
    }

    #[test]
    fn every_unusable_target_is_rejected_at_once() {
        // Caught here, before anything runs, and all reported together:
        // fixing one only to meet the next is a slow loop.
        let a = "when = \"at_completion\"\ntarget = \"agent.car.node.estimator.a\"\ncondition = \"equals\"\nvalue = 0.0";
        let b = "when = \"at_completion\"\ntarget = \"agent.car.node.estimator.b\"\ncondition = \"equals\"\nvalue = 0.0";
        let pipeline = empty_pipeline();
        let mut r = Runner::new(run("max_simulated_seconds = 5.0", &[a, b])).expect("valid run");
        let collector = r.collector();
        assert_eq!(
            r.on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &collector),
            Err(SetupError::UnusableAssertions(vec![
                UnusableAssertion {
                    number: 1,
                    target: target("agent.car.node.estimator.a"),
                    problems: vec![TargetProblem::Unreported],
                },
                UnusableAssertion {
                    number: 2,
                    target: target("agent.car.node.estimator.b"),
                    problems: vec![TargetProblem::Unreported],
                },
            ]))
        );
        assert_eq!(r.phase, Phase::Aborted);
    }

    #[test]
    fn setup_is_decided_once() {
        let (mut r, collector) = ready(
            run("max_simulated_seconds = 5.0", &[&term(CAR, 5.0)]),
            &[CAR],
        );
        let pipeline = pipeline();

        assert_eq!(
            r.on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &collector),
            Err(SetupError::AlreadySetUp)
        );
        assert!(matches!(r.phase, Phase::Ready(_)));
    }
}
