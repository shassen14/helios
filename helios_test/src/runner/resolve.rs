//! Checks, once the pipelines exist, that the run can judge every assertion:
//! its target names one series its agent records, a channel value there can
//! be extracted, a nondeterministic value is asserted on only with
//! permission, and the collector requests it. Each assertion that can be
//! judged binds its series to the exact key it names.
//!
//! Done against what each agent declares it reports, before the first tick,
//! so a run that could never pass stops before it starts and names every
//! problem at once. What the checks find is kept as the run's plan, so a
//! tick never looks a target up again, and the plan carries each assertion's
//! state, so only a run that can judge has any.

use crate::{
    assertion::{Assertion, AssertionTarget, TargetSubject},
    collector::{ObservationCollector, Series, SeriesId},
    runner::state::AssertionState,
};

use helios_core::prelude::AgentId;
use helios_runtime::{
    observe::{
        key::{RecordedKey, Subject},
        observation::ObservedValueKind,
        path::channel_path,
    },
    port::{ChannelKey, Determinism, Observable, Writer},
    AutonomyPipeline,
};

use std::fmt::Display;
use thiserror::Error;

/// Between the problems of one assertion. No problem's message holds it.
const PROBLEM_SEPARATOR: &str = " and ";

/// _Per-step._ What setup found for each assertion, and the state each
/// tick folds into it.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPlan {
    /// One per assertion, in run-file order.
    entries: Vec<PlanEntry>,
}

impl ResolvedPlan {
    /// Each assertion's series in `collector` beside its state, in run-file
    /// order. `None` when `collector` isn't the one the plan was resolved
    /// against.
    pub(super) fn with_series<'p, 'c>(
        &'p self,
        collector: &'c ObservationCollector,
    ) -> Option<Vec<(&'c Series, &'p AssertionState)>> {
        self.entries
            .iter()
            .map(|entry| Some((collector.series(entry.series)?, &entry.state)))
            .collect()
    }

    /// As [`with_series`](Self::with_series), with each assertion's number
    /// and its state open to update.
    pub(super) fn with_series_mut<'p, 'c>(
        &'p mut self,
        collector: &'c ObservationCollector,
    ) -> Option<Vec<(usize, &'c Series, &'p mut AssertionState)>> {
        self.entries
            .iter_mut()
            .map(|entry| {
                Some((
                    entry.number,
                    collector.series(entry.series)?,
                    &mut entry.state,
                ))
            })
            .collect()
    }

    /// Each assertion's state, in run-file order.
    #[cfg(test)]
    pub(super) fn states(&self) -> impl Iterator<Item = &AssertionState> {
        self.entries.iter().map(|entry| &entry.state)
    }
}

/// One assertion in the plan: its number, the series it is judged from,
/// and its state so far.
#[derive(Debug, Clone, PartialEq)]
struct PlanEntry {
    /// Its position in the run file, counting from 1, as in
    /// [`UnusableAssertion`].
    number: usize,
    series: SeriesId,
    state: AssertionState,
}

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
    /// Its agent reports nothing at that node and leaf, or has no channel
    /// by that name.
    #[error("its agent reports nothing there")]
    Unreported,
    /// A node path whose leaf is a channel that node writes. A channel is
    /// recorded under its own name, never its writer's, so the path to
    /// watch it is the one held here.
    #[error("a channel is not a node's leaf; to watch what the node writes there, use '{0}'")]
    ChannelUnderNode(String),
    /// A keyed channel written in several instances. The path names every
    /// instance, and an assertion judges one series.
    #[error("it names {0} instances of a keyed channel, and an assertion judges one series")]
    SeveralInstances(usize),
    /// A channel whose payload type has no extractor, so no value from it
    /// could be compared.
    #[error("it carries {0}, which has no extractor")]
    NoExtractor(&'static str),
    /// A nondeterministic leaf, asserted on without
    /// `allow_nondeterministic = true`.
    #[error("it is nondeterministic (set allow_nondeterministic = true to assert on it)")]
    Nondeterministic,
    /// The collector doesn't record it: the host built the collector from
    /// another run's assertions.
    #[error("the run's collector doesn't record it")]
    Unrequested,
}

/// What one agent reports, listed once for every assertion to search.
struct Catalog<'a> {
    agent: &'a AgentId,
    /// Each subject the agent's pipeline records, with what it records.
    observables: Vec<(Subject, Observable)>,
    /// Each channel a node writes, with the node's name.
    node_written: Vec<(&'a str, &'a ChannelKey)>,
}

impl Catalog<'_> {
    /// Every entry `subject`, a target's subject, names: at most one
    /// emission, or every instance of a channel. A target spells a channel
    /// by name alone, which is enough: the build refuses one name for two
    /// channels of an agent.
    fn matching(&self, subject: &TargetSubject) -> Vec<(&Subject, &Observable)> {
        self.observables
            .iter()
            .filter(|(entry, _)| match (subject, entry) {
                (
                    TargetSubject::Node { node, leaf },
                    Subject::Emission {
                        node: entry_node,
                        leaf: entry_leaf,
                    },
                ) => node == entry_node && leaf == entry_leaf,
                (TargetSubject::Channel { channel }, Subject::Channel { channel: key, .. }) => {
                    channel == key.name()
                }
                _ => false,
            })
            .map(|(entry, observable)| (entry, observable))
            .collect()
    }
}

/// The plan for judging `assertions`, with each one's series bound in
/// `collector`, or every one the run can't judge, in run-file order.
pub(super) fn resolve(
    assertions: &[Assertion],
    pipelines: &[(AgentId, &AutonomyPipeline)],
    collector: &mut ObservationCollector,
) -> Result<ResolvedPlan, Vec<UnusableAssertion>> {
    let catalogs: Vec<Catalog> = pipelines
        .iter()
        .map(|(agent, pipeline)| Catalog {
            agent,
            observables: pipeline.observables().collect(),
            node_written: pipeline
                .descriptions()
                .iter()
                .filter_map(|description| match description.writer() {
                    Writer::Node { node } => Some((node.as_str(), description.key())),
                    _ => None,
                })
                .collect(),
        })
        .collect();

    let mut entries = Vec::with_capacity(assertions.len());
    let mut unusable = Vec::new();
    for (index, assertion) in assertions.iter().enumerate() {
        let number = index + 1;
        match check(assertion, &catalogs, collector) {
            Ok(series) => entries.push(PlanEntry {
                number,
                series,
                state: AssertionState::new(),
            }),
            Err(problems) => unusable.push(UnusableAssertion {
                number,
                target: assertion.target().clone(),
                problems,
            }),
        }
    }

    if unusable.is_empty() {
        Ok(ResolvedPlan { entries })
    } else {
        Err(unusable)
    }
}

/// The series `assertion` is judged from, bound in `collector`, or every
/// reason the run can't judge it, checked against the one agent its target
/// names. A target that doesn't name exactly one series its agent records
/// has no other properties to check, so that problem is reported alone.
fn check(
    assertion: &Assertion,
    catalogs: &[Catalog],
    collector: &mut ObservationCollector,
) -> Result<SeriesId, Vec<TargetProblem>> {
    let target = assertion.target();
    let Some(catalog) = catalogs
        .iter()
        .find(|catalog| catalog.agent == target.agent())
    else {
        return Err(vec![TargetProblem::UnknownAgent]);
    };
    let (subject, observable) = match catalog.matching(target.subject()).as_slice() {
        [(subject, observable)] => (*subject, *observable),
        [] => return Err(vec![unreported(catalog, target.subject())]),
        several => return Err(vec![TargetProblem::SeveralInstances(several.len())]),
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
    if !collector.requests(target) {
        problems.push(TargetProblem::Unrequested);
    }

    if !problems.is_empty() {
        return Err(problems);
    }
    Ok(collector.bind(RecordedKey {
        agent: catalog.agent.clone(),
        subject: subject.clone(),
    }))
}

/// Why `subject` matched nothing `catalog` records: a channel spelled as a
/// leaf of the node that writes it, or nothing by that name.
fn unreported(catalog: &Catalog, subject: &TargetSubject) -> TargetProblem {
    match subject {
        TargetSubject::Node { node, leaf } => catalog
            .node_written
            .iter()
            .find(|(writer, channel)| *writer == node.as_ref() && channel.name() == leaf)
            .map(|(_, channel)| {
                TargetProblem::ChannelUnderNode(channel_path(catalog.agent.as_str(), channel))
            })
            .unwrap_or(TargetProblem::Unreported),
        TargetSubject::Channel { .. } => TargetProblem::Unreported,
    }
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
        observe::path::{channel_path, node_path},
        pipeline::autonomy_pipeline::TICK_DURATION_LEAF,
        port::{
            ChannelKey, ChannelKind, Keyed, PortBus, PortDescriptor, PortDescriptorBuilder,
            Provenance,
        },
        prelude::{PipelineNode, TickContext},
        BodyCapabilities, BodyChannel, PipelineBuilder,
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
    /// A keyed channel the planner writes in two instances.
    const EDGES: &str = "edges";
    /// A keyed channel the planner writes in one instance.
    const ROOT: &str = "root";
    /// A channel sent from outside the robot, which no node writes.
    const GOAL: &str = "goal";
    /// A channel the body publishes, which no node writes.
    const ODOMETRY: &str = "odometry";

    /// A value about one frame, keyed by the frame. No extractor reads it.
    struct FrameSample(&'static str);

    impl Keyed for FrameSample {
        type Instance = &'static str;

        fn instance(&self) -> &&'static str {
            &self.0
        }
    }

    fn keyed(name: &'static str) -> ChannelKey {
        ChannelKey::keyed::<FrameSample>(ChannelKind::Internal, name)
    }

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
    /// and [`LABEL`]; a planner writing [`SHARED`], two instances of
    /// [`EDGES`] and one of [`ROOT`]; [`ODOMETRY`] from the body; and
    /// [`GOAL`] sent from outside.
    fn pipeline() -> AutonomyPipeline {
        let estimator = PortDescriptorBuilder::algorithm()
            .outputs_from_slice(&[
                ChannelKey::internal::<f64>(SPEED),
                ChannelKey::internal::<String>(LABEL),
            ])
            .observable(NIS, Determinism::Deterministic)
            .observable(SHARED, Determinism::Deterministic)
            .build();
        let planner = PortDescriptorBuilder::algorithm()
            .outputs_from_slice(&[ChannelKey::internal::<String>(SHARED)])
            .output_keyed::<FrameSample>(keyed(EDGES), ["base", "wheel"])
            .output_keyed::<FrameSample>(keyed(ROOT), ["base"])
            .build();
        PipelineBuilder::new(AgentId::new(CAR))
            .with_body_capabilities(BodyCapabilities {
                publishes: vec![BodyChannel::new(
                    ChannelKey::internal::<f64>(ODOMETRY),
                    "wheel_encoder",
                    Provenance::Simulated,
                )],
                ..Default::default()
            })
            .with_outside_inputs(vec![ChannelKey::internal::<f64>(GOAL)])
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

    /// The car's key for `subject`.
    fn key(subject: Subject) -> RecordedKey {
        RecordedKey {
            agent: AgentId::new(CAR),
            subject,
        }
    }

    /// [`SPEED`] as the estimator declares it.
    fn speed_subject() -> Subject {
        Subject::channel(ChannelKey::internal::<f64>(SPEED), None)
    }

    fn target(agent: &str, node: &str, leaf: &str) -> AssertionTarget {
        AssertionTarget::parse(node_path(agent, node, leaf)).expect("valid target")
    }

    /// A target on the car's estimator.
    fn estimator(leaf: &str) -> AssertionTarget {
        target(CAR, ESTIMATOR, leaf)
    }

    /// A target on the car's channel named `name`. Only the name is spelled,
    /// so the key's type here is arbitrary.
    fn channel(name: &str) -> AssertionTarget {
        AssertionTarget::parse(channel_path(CAR, &ChannelKey::internal::<f64>(name)))
            .expect("valid target")
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

    /// `assertions` resolved against [`pipeline`] and a collector built for
    /// `requested`.
    fn resolve_with(
        assertions: &[Assertion],
        requested: &[AssertionTarget],
    ) -> (
        ObservationCollector,
        Result<ResolvedPlan, Vec<UnusableAssertion>>,
    ) {
        let pipeline = pipeline();
        let mut collector = ObservationCollector::new(requested, standard_extractors());
        let resolved = resolve(
            assertions,
            &[(AgentId::new(CAR), &pipeline)],
            &mut collector,
        );
        (collector, resolved)
    }

    /// The assertions the run can't judge, against a collector built for
    /// them.
    fn errors(assertions: &[Assertion]) -> Vec<UnusableAssertion> {
        let targets: Vec<AssertionTarget> = assertions.iter().map(|a| a.target().clone()).collect();
        resolve_with(assertions, &targets)
            .1
            .err()
            .unwrap_or_default()
    }

    #[test]
    fn the_plan_numbers_each_assertion_and_names_its_series_in_run_file_order() {
        // Two assertions on NIS share one series; SPEED has its own. Each
        // starts with a fresh state of its own.
        let nis = estimator(NIS);
        let speed = channel(SPEED);
        let (collector, resolved) = resolve_with(
            &[
                assertion(nis.clone(), false),
                assertion(speed.clone(), false),
                assertion(nis.clone(), true),
            ],
            &[nis.clone(), speed.clone()],
        );

        let entry = |number, subject: Subject| PlanEntry {
            number,
            series: collector
                .series_id(&key(subject))
                .expect("the assertion's key is bound"),
            state: AssertionState::new(),
        };
        assert_eq!(
            resolved,
            Ok(ResolvedPlan {
                entries: vec![
                    entry(1, Subject::emission(ESTIMATOR, NIS)),
                    entry(2, speed_subject()),
                    entry(3, Subject::emission(ESTIMATOR, NIS)),
                ],
            })
        );
    }

    #[test]
    fn a_channel_target_binds_the_key_its_agent_declares_payload_type_included() {
        // The path spells only the name; the bound key is the catalog's, so
        // a value of another payload type under that name lands nowhere.
        let (collector, resolved) =
            resolve_with(&[assertion(channel(SPEED), false)], &[channel(SPEED)]);

        assert!(resolved.is_ok());
        assert!(collector.series_id(&key(speed_subject())).is_some());
        assert_eq!(
            collector.series_id(&key(Subject::channel(
                ChannelKey::internal::<bool>(SPEED),
                None
            ))),
            None
        );
    }

    #[test]
    fn an_assertion_the_run_cannot_judge_binds_nothing() {
        let label = channel(LABEL);
        let (collector, resolved) = resolve_with(&[assertion(label.clone(), false)], &[label]);

        assert!(resolved.is_err());
        assert_eq!(
            collector.series_id(&key(Subject::channel(
                ChannelKey::internal::<String>(LABEL),
                None
            ))),
            None
        );
    }

    #[test]
    fn a_target_the_collector_does_not_record_is_unrequested() {
        // The collector was built for another run's assertions.
        let nis = estimator(NIS);
        let (_, resolved) = resolve_with(&[assertion(nis.clone(), false)], &[channel(SPEED)]);

        assert_eq!(
            resolved,
            Err(vec![unusable(1, nis, vec![TargetProblem::Unrequested])])
        );
    }

    #[test]
    fn a_declared_leaf_and_an_extractable_channel_resolve() {
        assert_eq!(
            errors(&[
                assertion(estimator(NIS), false),
                assertion(channel(SPEED), false),
            ]),
            vec![]
        );
    }

    #[test]
    fn a_channel_spelled_as_its_writers_leaf_is_refused_with_its_channel_path() {
        let speed = estimator(SPEED);

        assert_eq!(
            errors(&[assertion(speed.clone(), false)]),
            vec![unusable(
                1,
                speed,
                vec![TargetProblem::ChannelUnderNode(format!(
                    "agent.{CAR}.channel.{SPEED}"
                ))]
            )]
        );
    }

    #[test]
    fn a_channel_spelled_under_a_node_that_does_not_write_it_is_unreported() {
        let speed = target(CAR, PLANNER, SPEED);

        assert_eq!(
            errors(&[assertion(speed.clone(), false)]),
            vec![unusable(1, speed, vec![TargetProblem::Unreported])]
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
    fn a_channel_the_agent_has_no_writer_for_is_unreported() {
        let truth = channel("truth.tf");

        assert_eq!(
            errors(&[assertion(truth.clone(), false)]),
            vec![unusable(1, truth, vec![TargetProblem::Unreported])]
        );
    }

    #[test]
    fn a_body_channel_resolves_as_a_node_written_one_does() {
        assert_eq!(errors(&[assertion(channel(ODOMETRY), false)]), vec![]);
    }

    #[test]
    fn an_outside_channel_is_nondeterministic_and_resolves_with_permission() {
        let goal = channel(GOAL);

        assert_eq!(
            errors(&[
                assertion(goal.clone(), false),
                assertion(goal.clone(), true),
            ]),
            vec![unusable(1, goal, vec![TargetProblem::Nondeterministic])]
        );
    }

    #[test]
    fn a_keyed_channel_of_several_instances_is_refused_naming_the_count() {
        let edges = channel(EDGES);

        assert_eq!(
            errors(&[assertion(edges.clone(), false)]),
            vec![unusable(1, edges, vec![TargetProblem::SeveralInstances(2)])]
        );
    }

    #[test]
    fn a_keyed_channel_of_one_instance_is_checked_as_that_series() {
        // It names one series, so it gets as far as the extractor check.
        let root = channel(ROOT);

        assert_eq!(
            errors(&[assertion(root.clone(), false)]),
            vec![unusable(
                1,
                root,
                vec![TargetProblem::NoExtractor(
                    std::any::type_name::<FrameSample>()
                )]
            )]
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
    fn a_leaf_and_a_channel_of_one_name_are_told_apart() {
        // The estimator declares SHARED as a number; the planner writes a
        // SHARED channel with no extractor. The leaf resolves, the channel is
        // refused for its payload, and the channel spelled under the planner
        // is pointed to its own path.
        let on_channel = channel(SHARED);
        let on_planner = target(CAR, PLANNER, SHARED);

        assert_eq!(
            errors(&[
                assertion(estimator(SHARED), false),
                assertion(on_channel.clone(), false),
                assertion(on_planner.clone(), false),
            ]),
            vec![
                unusable(
                    2,
                    on_channel,
                    vec![TargetProblem::NoExtractor(std::any::type_name::<String>())]
                ),
                unusable(
                    3,
                    on_planner,
                    vec![TargetProblem::ChannelUnderNode(format!(
                        "agent.{CAR}.channel.{SHARED}"
                    ))]
                ),
            ]
        );
    }

    #[test]
    fn a_channel_whose_payload_has_no_extractor_is_refused() {
        let label = channel(LABEL);

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
        let label = channel(LABEL);
        let problems = vec![
            TargetProblem::Nondeterministic,
            TargetProblem::NoExtractor("String"),
        ];

        assert_eq!(
            unusable(4, label, problems).to_string(),
            "assertion 4 on 'agent.car.channel.label': it is nondeterministic \
             (set allow_nondeterministic = true to assert on it) and it carries String, \
             which has no extractor"
        );
    }
}
