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
    observe::observation::ObservedValueKind, port::Determinism, AutonomyPipeline,
};

use thiserror::Error;

/// _Pre-step._ Why the run can't judge an assertion.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum TargetError {
    /// No agent of that name, or its agent reports nothing there.
    #[error("'{0}' matches nothing its agent reports")]
    Unresolved(AssertionTarget),
    /// A channel whose payload type has no extractor, so no value from it
    /// could be compared.
    #[error("'{target}' carries {payload}, which has no extractor")]
    NoExtractor {
        target: AssertionTarget,
        payload: &'static str,
    },
    /// A nondeterministic leaf, asserted on without
    /// `allow_nondeterministic = true`.
    #[error("'{0}' is nondeterministic; set allow_nondeterministic = true to assert on it")]
    Nondeterministic(AssertionTarget),
}

/// Every reason the run can't judge one of `assertions`, in run-file order.
/// Empty when it can judge them all.
pub(super) fn resolve(
    assertions: &[Assertion],
    pipelines: &[(AgentId, &AutonomyPipeline)],
    collector: &ObservationCollector,
) -> Vec<TargetError> {
    assertions
        .iter()
        .filter_map(|assertion| check(assertion, pipelines, collector).err())
        .collect()
}

/// Checks one assertion against the one agent its target names.
fn check(
    assertion: &Assertion,
    pipelines: &[(AgentId, &AutonomyPipeline)],
    collector: &ObservationCollector,
) -> Result<(), TargetError> {
    let target = assertion.target();
    let node = target.node().as_ref();
    let leaf = target.leaf().as_ref();
    let unresolved = || TargetError::Unresolved(target.clone());

    let pipeline = pipelines
        .iter()
        .find(|(agent, _)| agent == target.agent())
        .map(|(_, pipeline)| *pipeline)
        .ok_or_else(unresolved)?;
    let observable = pipeline
        .observables()
        .find(|(name, observable)| *name == node && observable.leaf_name().as_ref() == leaf)
        .map(|(_, observable)| observable)
        .ok_or_else(unresolved)?;

    match observable.determinism() {
        Determinism::Deterministic => {}
        Determinism::Nondeterministic if assertion.allow_nondeterministic() => {}
        Determinism::Nondeterministic => return Err(TargetError::Nondeterministic(target.clone())),
    }

    match observable.kind() {
        ObservedValueKind::Scalar => Ok(()),
        ObservedValueKind::Typed { type_id, .. } if collector.can_extract(type_id) => Ok(()),
        ObservedValueKind::Typed { type_name, .. } => Err(TargetError::NoExtractor {
            target: target.clone(),
            payload: type_name,
        }),
    }
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
    /// A scalar leaf the estimator declares.
    const NIS: &str = "aiding.gps.nis";
    /// A channel the estimator writes whose payload has an extractor.
    const SPEED: &str = "speed";
    /// A channel the estimator writes whose payload has none.
    const LABEL: &str = "label";

    /// Writes nothing; exists only to carry a descriptor with outputs and
    /// leaves.
    struct DeclaringNode {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for DeclaringNode {
        fn name(&self) -> &str {
            ESTIMATOR
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _: &PortBus, _: &dyn TfProvider, _: TickContext) {}
    }

    /// One estimator declaring [`NIS`] and writing [`SPEED`] and [`LABEL`].
    fn pipeline() -> AutonomyPipeline {
        let descriptor = AlgorithmNodePortDescriptor::new()
            .outputs_from_slice(&[
                InternalChannel::named::<f64>(SPEED).into(),
                InternalChannel::named::<String>(LABEL).into(),
            ])
            .observable(NIS, Determinism::Deterministic)
            .build();
        PipelineBuilder::new()
            .add_node(Box::new(DeclaringNode { descriptor }))
            .build()
            .expect("the estimator builds alone")
    }

    fn target(agent: &str, leaf: &str) -> AssertionTarget {
        AssertionTarget::parse(agent_path(agent, ESTIMATOR, leaf)).expect("valid target")
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

    fn errors(assertions: &[Assertion]) -> Vec<TargetError> {
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
                assertion(target(CAR, NIS), false),
                assertion(target(CAR, SPEED), false),
            ]),
            vec![]
        );
    }

    #[test]
    fn a_leaf_its_agent_does_not_report_is_unresolved() {
        let missing = target(CAR, "missing");

        assert_eq!(
            errors(&[assertion(missing.clone(), false)]),
            vec![TargetError::Unresolved(missing)]
        );
    }

    #[test]
    fn a_leaf_only_another_agent_reports_is_unresolved() {
        let elsewhere = target("truck", NIS);

        assert_eq!(
            errors(&[assertion(elsewhere.clone(), false)]),
            vec![TargetError::Unresolved(elsewhere)]
        );
    }

    #[test]
    fn a_tap_whose_payload_has_no_extractor_is_refused() {
        let label = target(CAR, LABEL);

        assert_eq!(
            errors(&[assertion(label.clone(), false)]),
            vec![TargetError::NoExtractor {
                target: label,
                payload: std::any::type_name::<String>(),
            }]
        );
    }

    #[test]
    fn a_nondeterministic_leaf_is_refused_without_permission() {
        let duration = target(CAR, TICK_DURATION_LEAF);

        assert_eq!(
            errors(&[assertion(duration.clone(), false)]),
            vec![TargetError::Nondeterministic(duration)]
        );
    }

    #[test]
    fn a_nondeterministic_leaf_is_accepted_with_permission() {
        assert_eq!(
            errors(&[assertion(target(CAR, TICK_DURATION_LEAF), true)]),
            vec![]
        );
    }

    #[test]
    fn permission_on_a_deterministic_leaf_is_accepted() {
        assert_eq!(errors(&[assertion(target(CAR, NIS), true)]), vec![]);
    }

    #[test]
    fn every_problem_is_reported_in_run_file_order() {
        let missing = target(CAR, "missing");
        let duration = target(CAR, TICK_DURATION_LEAF);

        assert_eq!(
            errors(&[
                assertion(missing.clone(), false),
                assertion(target(CAR, NIS), false),
                assertion(duration.clone(), false),
            ]),
            vec![
                TargetError::Unresolved(missing),
                TargetError::Nondeterministic(duration),
            ]
        );
    }
}
