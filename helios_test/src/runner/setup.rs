use super::{
    resolve::{resolve, RunEvaluations, Unresolved, Unusable},
    Abort, Phase, Runner,
};
use crate::{collector::ObservationCollector, report::HostFault};
use helios_core::prelude::AgentId;
use helios_runtime::AutonomyPipeline;

use std::fmt::{self, Display};
use thiserror::Error;

/// Between the agents a refusal lists.
const AGENT_SEPARATOR: &str = ", ";

/// Between the unbuilt agents and the unusable entries in a refusal.
const REFUSAL_SEPARATOR: &str = "; ";

impl Runner {
    /// Check, before the first tick, that the run can build every
    /// evaluation and evaluate every assertion and measure against what
    /// `pipelines` report and what `collector` requests, bind each one's
    /// series and inputs in `collector`, and keep what the check found for
    /// every tick. The run is then ready, or, if any can't be, aborted with
    /// the reasons. An agent in `unbuilt` aborts the run, and every entry
    /// and evaluation on it is refused for that, beside every other
    /// refusal, so one run shows every problem.
    ///
    /// Called once, before the host asks `collector` what to record: the
    /// request is what setup binds. A second call, or one after setup
    /// failed, is refused and changes nothing. A collector the host has
    /// already asked would record nothing setup binds, so the run is
    /// aborted.
    pub fn on_pipeline_built(
        &mut self,
        pipelines: &[(AgentId, &AutonomyPipeline)],
        unbuilt: &[UnbuiltPipeline],
        collector: &mut ObservationCollector,
    ) -> Result<(), SetupError> {
        if self.phase != Phase::AwaitingPipelines {
            return Err(SetupError::AlreadySetUp);
        }
        if collector.was_asked() {
            self.phase = Phase::Aborted(Abort::fault(HostFault::AskedBeforeSetup));
            return Err(SetupError::AskedBeforeSetup);
        }

        let evaluations = RunEvaluations {
            inline: self.run.evaluations(),
            defaults: self.run.evaluation_defaults(),
            suites: self.run.suites(),
        };
        let unbuilt_agents: Vec<AgentId> = unbuilt
            .iter()
            .map(|unbuilt| unbuilt.agent.clone())
            .collect();
        let resolved = resolve(
            evaluations,
            self.run.assertions(),
            self.run.measures(),
            &self.evaluators,
            pipelines,
            &unbuilt_agents,
            collector,
        );
        let (bound, unusable) = match resolved {
            Ok(plan) if unbuilt.is_empty() => {
                self.phase = Phase::Ready(plan);
                return Ok(());
            }
            Ok(plan) => (plan.into_evaluations(), Unusable::default()),
            Err(Unresolved { bound, unusable }) => (bound, unusable),
        };
        self.phase = Phase::Aborted(Abort {
            unbuilt: unbuilt.to_vec(),
            unusable: unusable.clone(),
            bound,
            fault: None,
        });
        Err(SetupError::Refused(Refusal {
            unbuilt: unbuilt_agents,
            unusable,
        }))
    }
}

/// _Pre-step._ An agent whose pipeline the host couldn't build, with every
/// error it reported.
#[derive(Debug, Clone, PartialEq)]
pub struct UnbuiltPipeline {
    pub agent: AgentId,
    /// As the host rendered them.
    pub errors: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Error)]
pub enum SetupError {
    /// Agents whose pipeline wasn't built, and evaluations, assertions and
    /// measures the run can't evaluate, each with its reasons, so the
    /// operator sees every one to fix at once.
    #[error("{0}")]
    Refused(Refusal),
    /// `on_pipeline_built` was called after the run's setup was decided.
    #[error("the run's setup is already decided")]
    AlreadySetUp,
    /// The host asked the collector what to record before setup bound
    /// anything, so it would record nothing the run reads.
    #[error(
        "the host asked the collector what to record before setup; set the runner up once the \
         pipelines exist, then ask its sinks"
    )]
    AskedBeforeSetup,
}

/// _Pre-step._ Why setup refused a run: the agents whose pipeline wasn't
/// built, and what it can't evaluate. One or both is never empty.
#[derive(Debug, Clone, PartialEq)]
pub struct Refusal {
    pub unbuilt: Vec<AgentId>,
    pub unusable: Unusable,
}

impl Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut parts = Vec::new();
        if !self.unbuilt.is_empty() {
            let agents: Vec<&str> = self.unbuilt.iter().map(AgentId::as_str).collect();
            parts.push(format!(
                "these agents' pipelines weren't built: {}",
                agents.join(AGENT_SEPARATOR)
            ));
        }
        if !self.unusable.is_empty() {
            parts.push(format!(
                "the run can't use these evaluations, assertions and measures: {}",
                self.unusable
            ));
        }
        f.write_str(&parts.join(REFUSAL_SEPARATOR))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::{AssertionTarget, EntryName},
        runner::{
            fixtures::{
                lag, lag_target, pipeline, ready, run, run_evaluating, set_up, set_up_unbuilt,
                speed_target, speed_term, term, CAR, TRUCK,
            },
            resolve::evaluations::EvaluationProblem,
            resolve::{TargetProblem, UnusableEntry},
        },
    };

    use helios_runtime::{
        observe::sink::{ObservationSink, WatchRequest},
        PipelineBuilder,
    };

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
        assert_eq!(r.on_pipeline_built(&[], &[], &mut collector), Ok(()));
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
        let unusable = Unusable {
            evaluations: vec![],
            entries: vec![
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
            ],
        };
        assert_eq!(
            r.on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &[], &mut collector),
            Err(SetupError::Refused(Refusal {
                unbuilt: vec![],
                unusable: unusable.clone(),
            }))
        );
        assert_eq!(
            r.phase,
            Phase::Aborted(Abort {
                unusable,
                ..Abort::default()
            })
        );
    }

    #[test]
    fn an_unbuilt_agent_aborts_the_run_and_refuses_what_reads_it() {
        let (r, _, setup) = set_up_unbuilt(
            run("max_simulated_seconds = 5.0", &[&term("a", TRUCK, 5.0)]),
            &[CAR],
            &[TRUCK],
        );

        let Err(SetupError::Refused(refusal)) = setup else {
            panic!("the run is refused: {setup:?}");
        };
        assert_eq!(refusal.unbuilt, [AgentId::new(TRUCK)]);
        assert_eq!(
            refusal.unusable.entries[0].problems,
            [TargetProblem::PipelineNotBuilt]
        );
        assert!(
            refusal
                .to_string()
                .starts_with("these agents' pipelines weren't built: truck; the run can't use"),
            "{refusal}"
        );
        assert!(r.is_aborted());
    }

    #[test]
    fn an_unbuilt_agent_aborts_a_run_that_reads_nothing_of_it() {
        let (r, _, setup) =
            set_up_unbuilt(run("max_simulated_seconds = 5.0", &[]), &[CAR], &[TRUCK]);

        assert_eq!(
            setup,
            Err(SetupError::Refused(Refusal {
                unbuilt: vec![AgentId::new(TRUCK)],
                unusable: Unusable::default(),
            }))
        );
        assert!(r.is_aborted());
    }

    #[test]
    fn setup_is_decided_once() {
        let (mut r, mut collector) = ready(
            run("max_simulated_seconds = 5.0", &[&term("a", CAR, 5.0)]),
            &[CAR],
        );
        let pipeline = pipeline();

        assert_eq!(
            r.on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &[], &mut collector),
            Err(SetupError::AlreadySetUp)
        );
        assert!(matches!(r.phase, Phase::Ready(_)));
    }

    #[test]
    fn a_collector_asked_before_setup_aborts_the_run() {
        let pipeline = pipeline();
        let mut r = Runner::new(run("max_simulated_seconds = 5.0", &[&term("a", CAR, 5.0)]))
            .expect("valid run");
        let mut collector = r.collector();
        collector.request();

        assert_eq!(
            r.on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &[], &mut collector),
            Err(SetupError::AskedBeforeSetup)
        );
        assert_eq!(
            r.phase,
            Phase::Aborted(Abort::fault(HostFault::AskedBeforeSetup))
        );
    }

    #[test]
    fn the_request_asked_after_setup_is_what_setup_bound() {
        let (_, collector) = ready(
            run("max_simulated_seconds = 5.0", &[&speed_term("a", CAR, 5.0)]),
            &[CAR],
        );

        assert_eq!(
            collector.request(),
            WatchRequest::Paths([speed_target(CAR)].into())
        );
    }

    /// A `lag` evaluation named `name` with its inputs as written.
    fn lag_with(name: &str, actual: &str, wanted: &str) -> String {
        format!(
            "name = \"{name}\"\nkind = \"lag\"\n[evaluate.inputs]\n\
             actual = {actual}\nwanted = {wanted}"
        )
    }

    /// A measure named `name` of `target`'s mean.
    fn mean_of(name: &str, target: &str) -> String {
        format!("name = \"{name}\"\ntarget = {target}\nreduce = \"mean\"")
    }

    /// What setup refused, for a run of `evaluations` and `measures` on the
    /// car and the truck.
    fn refused(evaluations: &[&str], measures: &[&str]) -> Unusable {
        let run = run_evaluating("max_simulated_seconds = 5.0", &[], measures, evaluations);
        match set_up(run, &[CAR, TRUCK]).2 {
            Err(SetupError::Refused(refusal)) => refusal.unusable,
            other => panic!("expected setup to refuse the run, got {other:?}"),
        }
    }

    const WHEELS: &str = "\"agent.car.channel.wheels\"";

    #[test]
    fn a_run_whose_evaluation_can_be_built_is_ready() {
        let (r, _) = ready(
            run_evaluating(
                "max_simulated_seconds = 5.0",
                &[],
                &[&mean_of("m", &format!("\"{}\"", lag_target("slow")))],
                &[&lag("slow", CAR)],
            ),
            &[CAR],
        );
        assert!(matches!(r.phase, Phase::Ready(_)));
    }

    #[test]
    fn an_input_of_another_payload_type_is_refused_naming_both() {
        let wheel = format!("{{ path = {WHEELS}, instance = \"front\" }}");
        let unusable = refused(&[&lag_with("slow", &wheel, &wheel)], &[]);

        let [evaluation] = unusable.evaluations.as_slice() else {
            panic!("one refused evaluation: {unusable}");
        };
        assert!(matches!(
            &evaluation.problems[..],
            [EvaluationProblem::Input {
                role: "actual",
                problem: TargetProblem::WrongPayload { wants: "f64", .. },
                ..
            }]
        ));
    }

    #[test]
    fn an_input_on_every_instance_is_refused() {
        let speed = format!("\"{}\"", speed_target(CAR));
        let every = format!("{{ path = {WHEELS}, instance = \"*\" }}");
        let unusable = refused(&[&lag_with("slow", &speed, &every)], &[]);

        assert!(matches!(
            &unusable.evaluations[0].problems[..],
            [EvaluationProblem::Input {
                problem: TargetProblem::InputOnSeveral(_),
                ..
            }]
        ));
    }

    #[test]
    fn a_gap_given_to_a_role_with_no_cap_is_refused_naming_the_role() {
        // Lag holds its wanted speed however old; the base's gap is used,
        // to find where the speeds dropped out.
        let gaps = format!("{}\n[evaluate.gap.wanted]\nseconds = 0.5", lag("slow", CAR));
        let unusable = refused(&[&gaps], &[]);

        assert_eq!(
            unusable.evaluations[0].problems,
            vec![EvaluationProblem::UnusedGap("wanted")]
        );
    }

    #[test]
    fn a_base_with_no_cap_to_find_its_gaps_by_is_refused_naming_the_remedies() {
        // The speed declares no rate, and the run gives no gap.
        let speed = format!("\"{}\"", speed_target(CAR));
        let front = format!("{{ path = {WHEELS}, instance = \"front\" }}");
        let unusable = refused(&[&lag_with("slow", &speed, &front)], &[]);

        assert_eq!(
            unusable.evaluations[0].problems,
            vec![EvaluationProblem::Refused(
                "the actual is the base, whose gaps are found against a cap, and none is \
                 given: give [evaluation] gap_multiple in the entry's file, or \
                 [evaluate.gap.actual] multiple or seconds"
                    .to_owned()
            )]
        );
    }

    #[test]
    fn an_entry_on_a_refused_evaluation_is_refused_for_that_alone() {
        // The kind refuses inputs from two agents; the measure on its
        // metric is refused because its evaluation is.
        let speed = format!("\"{}\"", speed_target(TRUCK));
        let front = format!("{{ path = {WHEELS}, instance = \"front\" }}");
        let derived = format!("\"{}\"", lag_target("slow"));
        let unusable = refused(
            &[&lag_with("slow", &speed, &front)],
            &[&mean_of("m", &derived)],
        );

        assert_eq!(
            unusable.evaluations[0].problems,
            [EvaluationProblem::Refused(
                "lag compares one agent's speeds".to_string()
            )]
        );
        assert_eq!(
            unusable.entries[0].problems,
            [TargetProblem::EvaluationUnusable]
        );
        let message = SetupError::Refused(Refusal {
            unbuilt: vec![],
            unusable,
        })
        .to_string();
        assert!(
            message.contains("evaluation 'slow': lag compares one agent's speeds; 'm'"),
            "{message}"
        );
    }

    #[test]
    fn an_instance_on_a_derived_series_is_refused() {
        let derived = format!(
            "{{ path = \"{}\", instance = \"front\" }}",
            lag_target("slow")
        );
        let unusable = refused(&[&lag("slow", CAR)], &[&mean_of("m", &derived)]);

        assert!(unusable.evaluations.is_empty());
        assert_eq!(
            unusable.entries[0].problems,
            [TargetProblem::InstanceOnDerived]
        );
    }

    #[test]
    fn a_metric_its_bound_evaluation_doesnt_offer_is_refused_naming_what_it_does() {
        // lag's menu is known only once bound, so load let the name through.
        let unusable = refused(
            &[&lag("slow", CAR)],
            &[&mean_of("m", "\"eval.inline.slow.excess\"")],
        );

        assert!(unusable.evaluations.is_empty());
        assert_eq!(
            unusable.entries[0].problems,
            [TargetProblem::UnofferedMetric(vec!["shortfall"])]
        );
        assert!(
            unusable.entries[0].problems[0]
                .to_string()
                .ends_with("(it offers shortfall)"),
            "{:?}",
            unusable.entries[0].problems
        );
    }
}
