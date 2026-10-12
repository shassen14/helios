//! Checks, once the pipelines exist, that the run can judge every assertion:
//! its target is a leaf its agent reports, a channel value there can be
//! extracted, and a nondeterministic leaf is asserted on only with
//! permission.
//!
//! Done against what each agent declares it reports, before the first tick,
//! so a run that could never pass stops before it starts and names every
//! problem at once.

use crate::{
    assertion::{Assertion, AssertionTarget},
    collector::ObservationCollector,
};

use helios_core::prelude::AgentId;
use helios_runtime::{
    observe::observation::ObservedValueKind,
    port::{Determinism, Observable},
    AutonomyPipeline,
};

use std::fmt::Display;
use thiserror::Error;

/// Between the problems of one assertion. No problem's message holds it.
const PROBLEM_SEPARATOR: &str = " and ";

/// _Pre-step._ An assertion the run can't judge, and every reason why.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error(
    "assertion {number} on '{target}': {}",
    join(.problems, PROBLEM_SEPARATOR)
)]
pub struct UnusableAssertion {
    /// Its position in the run file, counting from 1, so two assertions on
    /// one target can be told apart.
    pub(super) number: usize,
    pub(super) target: AssertionTarget,
    pub(super) problems: Vec<TargetProblem>,
}

/// _Pre-step._ One reason the run can't judge an assertion.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum TargetProblem {
    /// The scenario has no agent of that name with a pipeline.
    #[error("no agent by that name has a pipeline")]
    UnknownAgent,
    /// Its agent reports nothing at that node and leaf.
    #[error("its agent reports nothing there")]
    Unreported,
    /// A channel whose payload type has no extractor, so no value from it
    /// could be compared.
    #[error("it carries {0}, which has no extractor")]
    NoExtractor(&'static str),
    /// A nondeterministic leaf, asserted on without
    /// `allow_nondeterministic = true`.
    #[error("it is nondeterministic (set allow_nondeterministic = true to assert on it)")]
    Nondeterministic,
}

/// What one agent reports, listed once for every assertion to search.
struct Catalog<'a> {
    agent: &'a AgentId,
    /// Each leaf with the name of the node that reports it.
    observables: Vec<(&'a str, Observable)>,
}

/// Every assertion in `assertions` the run can't judge, in run-file order.
/// Empty when it can judge them all.
pub(super) fn resolve(
    assertions: &[Assertion],
    pipelines: &[(AgentId, &AutonomyPipeline)],
    collector: &ObservationCollector,
) -> Vec<UnusableAssertion> {
    let catalogs: Vec<Catalog> = pipelines
        .iter()
        .map(|(agent, pipeline)| Catalog {
            agent,
            observables: pipeline.observables().collect(),
        })
        .collect();

    assertions
        .iter()
        .enumerate()
        .filter_map(|(index, assertion)| {
            let problems = check(assertion, &catalogs, collector);
            (!problems.is_empty()).then(|| UnusableAssertion {
                number: index + 1,
                target: assertion.target().clone(),
                problems,
            })
        })
        .collect()
}

/// Every reason the run can't judge `assertion`, checked against the one
/// agent its target names. A target its agent doesn't report has no other
/// properties to check, so that problem is reported alone.
fn check(
    assertion: &Assertion,
    catalogs: &[Catalog],
    collector: &ObservationCollector,
) -> Vec<TargetProblem> {
    let target = assertion.target();
    let node = target.node().as_ref();
    let leaf = target.leaf().as_ref();

    let Some(catalog) = catalogs
        .iter()
        .find(|catalog| catalog.agent == target.agent())
    else {
        return vec![TargetProblem::UnknownAgent];
    };
    let Some(observable) = catalog
        .observables
        .iter()
        .find(|(name, observable)| *name == node && observable.leaf_name().as_ref() == leaf)
        .map(|(_, observable)| observable)
    else {
        return vec![TargetProblem::Unreported];
    };

    let mut problems = Vec::new();
    match observable.determinism() {
        Determinism::Deterministic => {}
        Determinism::Nondeterministic if assertion.allow_nondeterministic() => {}
        Determinism::Nondeterministic => problems.push(TargetProblem::Nondeterministic),
    }
    match observable.kind() {
        ObservedValueKind::Scalar => {}
        ObservedValueKind::Typed { type_id, .. } if collector.can_extract(type_id) => {}
        ObservedValueKind::Typed { type_name, .. } => {
            problems.push(TargetProblem::NoExtractor(type_name))
        }
    }
    problems
}

/// `items` on one line, `separator` between each.
pub(super) fn join(items: &[impl Display], separator: &str) -> String {
    items
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(separator)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::{
        extract::standard_extractors, terminal::TerminalAssertion, AssertionValue, Condition,
    };

    use helios_core::prelude::TfProvider;
    use helios_runtime::{
        observe::path::agent_path,
        pipeline::autonomy_pipeline::TICK_DURATION_LEAF,
        port::{AlgorithmNodePortDescriptor, InternalChannel, PortBus, PortDescriptor},
        prelude::{PipelineNode, TickContext},
        PipelineBuilder,
    };

    const CAR: &str = "car";
    const ESTIMATOR: &str = "estimator";
    const PLANNER: &str = "planner";
    /// A scalar leaf the estimator declares.
    const NIS: &str = "aiding.gps.nis";
    /// A channel the estimator writes whose payload has an extractor.
    const SPEED: &str = "speed";
    /// A channel the estimator writes whose payload has none.
    const LABEL: &str = "label";
    /// A scalar leaf the estimator declares, and a channel the planner writes
    /// whose payload has no extractor.
    const SHARED: &str = "shared";

    /// Writes nothing; exists only to carry a name and a descriptor with
    /// outputs and leaves.
    struct DeclaringNode {
        name: &'static str,
        descriptor: PortDescriptor,
    }

    impl PipelineNode for DeclaringNode {
        fn name(&self) -> &str {
            self.name
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _: &PortBus, _: &dyn TfProvider, _: TickContext) {}
    }

    /// An estimator declaring [`NIS`] and [`SHARED`] and writing [`SPEED`]
    /// and [`LABEL`], and a planner writing [`SHARED`].
    fn pipeline() -> AutonomyPipeline {
        let estimator = AlgorithmNodePortDescriptor::new()
            .outputs_from_slice(&[
                InternalChannel::named::<f64>(SPEED).into(),
                InternalChannel::named::<String>(LABEL).into(),
            ])
            .observable(NIS, Determinism::Deterministic)
            .observable(SHARED, Determinism::Deterministic)
            .build();
        let planner = AlgorithmNodePortDescriptor::new()
            .outputs_from_slice(&[InternalChannel::named::<String>(SHARED).into()])
            .build();
        PipelineBuilder::new()
            .add_node(Box::new(DeclaringNode {
                name: ESTIMATOR,
                descriptor: estimator,
            }))
            .add_node(Box::new(DeclaringNode {
                name: PLANNER,
                descriptor: planner,
            }))
            .build()
            .expect("two nodes with no inputs build")
    }

    fn target(agent: &str, node: &str, leaf: &str) -> AssertionTarget {
        AssertionTarget::parse(agent_path(agent, node, leaf)).expect("valid target")
    }

    /// A target on the car's estimator.
    fn estimator(leaf: &str) -> AssertionTarget {
        target(CAR, ESTIMATOR, leaf)
    }

    fn assertion(target: AssertionTarget, allow_nondeterministic: bool) -> Assertion {
        Assertion::Terminal(TerminalAssertion {
            target,
            condition: Condition::Equals,
            value: AssertionValue::Float(0.0),
            low: None,
            high: None,
            allow_nondeterministic,
        })
    }

    fn unusable(
        number: usize,
        target: AssertionTarget,
        problems: Vec<TargetProblem>,
    ) -> UnusableAssertion {
        UnusableAssertion {
            number,
            target,
            problems,
        }
    }

    fn errors(assertions: &[Assertion]) -> Vec<UnusableAssertion> {
        let pipeline = pipeline();
        let collector = ObservationCollector::new(
            assertions.iter().map(Assertion::target),
            standard_extractors(),
        );
        resolve(assertions, &[(AgentId::new(CAR), &pipeline)], &collector)
    }

    #[test]
    fn a_declared_leaf_and_an_extractable_tap_resolve() {
        assert_eq!(
            errors(&[
                assertion(estimator(NIS), false),
                assertion(estimator(SPEED), false),
            ]),
            vec![]
        );
    }

    #[test]
    fn a_leaf_its_agent_does_not_report_is_unreported() {
        let missing = estimator("missing");

        assert_eq!(
            errors(&[assertion(missing.clone(), false)]),
            vec![unusable(1, missing, vec![TargetProblem::Unreported])]
        );
    }

    #[test]
    fn an_agent_with_no_pipeline_is_unknown() {
        let elsewhere = target("truck", ESTIMATOR, NIS);

        assert_eq!(
            errors(&[assertion(elsewhere.clone(), false)]),
            vec![unusable(1, elsewhere, vec![TargetProblem::UnknownAgent])]
        );
    }

    #[test]
    fn a_leaf_is_checked_on_the_node_the_target_names() {
        // The estimator declares SHARED as a number; the planner's SHARED is
        // a tap with no extractor. Only the planner's is refused.
        let on_planner = target(CAR, PLANNER, SHARED);

        assert_eq!(
            errors(&[
                assertion(estimator(SHARED), false),
                assertion(on_planner.clone(), false),
            ]),
            vec![unusable(
                2,
                on_planner,
                vec![TargetProblem::NoExtractor(std::any::type_name::<String>())]
            )]
        );
    }

    #[test]
    fn a_tap_whose_payload_has_no_extractor_is_refused() {
        let label = estimator(LABEL);

        assert_eq!(
            errors(&[assertion(label.clone(), false)]),
            vec![unusable(
                1,
                label,
                vec![TargetProblem::NoExtractor(std::any::type_name::<String>())]
            )]
        );
    }

    #[test]
    fn a_nondeterministic_leaf_is_refused_without_permission() {
        let duration = estimator(TICK_DURATION_LEAF);

        assert_eq!(
            errors(&[assertion(duration.clone(), false)]),
            vec![unusable(1, duration, vec![TargetProblem::Nondeterministic])]
        );
    }

    #[test]
    fn a_nondeterministic_leaf_is_accepted_with_permission() {
        assert_eq!(
            errors(&[assertion(estimator(TICK_DURATION_LEAF), true)]),
            vec![]
        );
    }

    #[test]
    fn permission_on_a_deterministic_leaf_is_accepted() {
        assert_eq!(errors(&[assertion(estimator(NIS), true)]), vec![]);
    }

    #[test]
    fn two_assertions_on_one_target_are_judged_apart() {
        // Permission is per assertion, so only the one without it is
        // refused, and its number says which.
        let duration = estimator(TICK_DURATION_LEAF);

        assert_eq!(
            errors(&[
                assertion(duration.clone(), true),
                assertion(duration.clone(), false),
            ]),
            vec![unusable(2, duration, vec![TargetProblem::Nondeterministic])]
        );
    }

    #[test]
    fn every_unusable_assertion_is_reported_in_run_file_order() {
        let missing = estimator("missing");
        let duration = estimator(TICK_DURATION_LEAF);

        assert_eq!(
            errors(&[
                assertion(missing.clone(), false),
                assertion(estimator(NIS), false),
                assertion(duration.clone(), false),
            ]),
            vec![
                unusable(1, missing, vec![TargetProblem::Unreported]),
                unusable(3, duration, vec![TargetProblem::Nondeterministic]),
            ]
        );
    }

    #[test]
    fn the_message_names_the_assertion_its_target_and_every_problem() {
        let label = estimator(LABEL);
        let problems = vec![
            TargetProblem::Nondeterministic,
            TargetProblem::NoExtractor("String"),
        ];

        assert_eq!(
            unusable(4, label, problems).to_string(),
            "assertion 4 on 'agent.car.estimator.label': it is nondeterministic \
             (set allow_nondeterministic = true to assert on it) and it carries String, \
             which has no extractor"
        );
    }
}
