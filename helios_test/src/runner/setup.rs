use super::{
    resolve::{join, resolve, UnusableEntry},
    Phase, Runner,
};
use crate::collector::ObservationCollector;
use helios_core::prelude::AgentId;
use helios_runtime::AutonomyPipeline;

use thiserror::Error;

impl Runner {
    /// Check, before the first tick, that the run can evaluate every
    /// assertion and measure against what `pipelines` report and what
    /// `collector` requests, bind each one's series in `collector`, and keep
    /// what the check found for every tick. The run is then ready, or, if
    /// any entry can't be evaluated, aborted with the reasons.
    ///
    /// Called once. A second call, or one after setup failed, is refused and
    /// changes nothing.
    pub fn on_pipeline_built(
        &mut self,
        pipelines: &[(AgentId, &AutonomyPipeline)],
        collector: &mut ObservationCollector,
    ) -> Result<(), SetupError> {
        if self.phase != Phase::AwaitingPipelines {
            return Err(SetupError::AlreadySetUp);
        }

        match resolve(
            self.run.assertions(),
            self.run.measures(),
            pipelines,
            collector,
        ) {
            Ok(plan) => {
                self.phase = Phase::Ready(plan);
                Ok(())
            }
            Err(unusable) => {
                self.phase = Phase::Aborted;
                Err(SetupError::UnusableEntries(unusable))
            }
        }
    }
}

/// Between the unusable entries in one message. An entry's own message
/// holds a `,` but never this.
const ENTRY_SEPARATOR: &str = "; ";

#[derive(Debug, Clone, PartialEq, Error)]
pub enum SetupError {
    /// Assertions and measures the run can't evaluate, each with its
    /// reasons, so the operator sees every one to fix at once.
    #[error(
        "the run can't evaluate these assertions and measures: {}",
        join(.0, ENTRY_SEPARATOR)
    )]
    UnusableEntries(Vec<UnusableEntry>),
    /// `on_pipeline_built` was called after the run's setup was decided.
    #[error("the run's setup is already decided")]
    AlreadySetUp,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::{AssertionTarget, EntryName},
        runner::{
            fixtures::{pipeline, ready, run, term, CAR},
            resolve::TargetProblem,
        },
    };

    use helios_runtime::PipelineBuilder;

    // A node-less pipeline: its agent reports nothing, so every target on it
    // is unreported. What resolves is tested beside `resolve`.
    fn empty_pipeline() -> AutonomyPipeline {
        PipelineBuilder::new(AgentId::new(CAR))
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
        let mut collector = r.collector();
        assert_eq!(r.on_pipeline_built(&[], &mut collector), Ok(()));
    }

    #[test]
    fn a_run_whose_assertions_can_all_be_judged_is_ready() {
        let (r, _) = ready(
            run("max_simulated_seconds = 5.0", &[&term("a", CAR, 5.0)]),
            &[CAR],
        );
        assert!(matches!(r.phase, Phase::Ready(_)));
    }

    #[test]
    fn every_unusable_target_is_rejected_at_once() {
        // Caught here, before anything runs, and all reported together:
        // fixing one only to meet the next is a slow loop.
        let a = "name = \"a\"\ntarget = \"agent.car.node.estimator.a\"\nreduce = \"final\"\ncondition = { less_than = 1.0 }";
        let b = "name = \"b\"\ntarget = \"agent.car.node.estimator.b\"\nreduce = \"final\"\ncondition = { less_than = 1.0 }";
        let pipeline = empty_pipeline();
        let mut r = Runner::new(run("max_simulated_seconds = 5.0", &[a, b])).expect("valid run");
        let mut collector = r.collector();
        assert_eq!(
            r.on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &mut collector),
            Err(SetupError::UnusableEntries(vec![
                UnusableEntry {
                    name: EntryName::parse("a").expect("valid name"),
                    target: target("agent.car.node.estimator.a"),
                    problems: vec![TargetProblem::Unreported],
                },
                UnusableEntry {
                    name: EntryName::parse("b").expect("valid name"),
                    target: target("agent.car.node.estimator.b"),
                    problems: vec![TargetProblem::Unreported],
                },
            ]))
        );
        assert_eq!(r.phase, Phase::Aborted);
    }

    #[test]
    fn setup_is_decided_once() {
        let (mut r, mut collector) = ready(
            run("max_simulated_seconds = 5.0", &[&term("a", CAR, 5.0)]),
            &[CAR],
        );
        let pipeline = pipeline();

        assert_eq!(
            r.on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &mut collector),
            Err(SetupError::AlreadySetUp)
        );
        assert!(matches!(r.phase, Phase::Ready(_)));
    }
}
