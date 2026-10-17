//! Checks, once the pipelines exist, that the run can evaluate every
//! assertion and measure: its target names one series its agent records, or
//! each instance of a keyed channel it asks for, and each series its window
//! is anchored `after` names one; a channel value there can be extracted, a
//! nondeterministic value is read only with permission, and the collector
//! requests it; a reducer that holds values between samples isn't given an
//! emission's events; and the reducer and the condition can apply to the
//! kind of value the series holds. Each entry that can be evaluated binds
//! its series, one per instance, and its anchors', to the exact keys they
//! name.
//!
//! Done against what each agent declares it reports, before the first tick,
//! so a run that could never pass stops before it starts and names every
//! problem at once. What the checks find is kept as the run's plan, so a
//! tick never looks a target up again, and the plan carries each entry's
//! state, so only a run that can judge has any.

use crate::{
    assertion::{
        condition::KindProblem, Assertion, AssertionTarget, Condition, EntryName, InstanceSelector,
        Measure, TargetSubject, ValueKind, Window,
    },
    collector::{instance_of, ObservationCollector, Series, SeriesId},
    runner::state::AssertionState,
    statistics::within_run::{ReduceError, Reducer},
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

use std::{collections::HashMap, fmt::Display};
use thiserror::Error;

/// Between the problems of one assertion. No problem's message holds it.
const PROBLEM_SEPARATOR: &str = " and ";

/// Between the instances a problem lists.
const INSTANCE_SEPARATOR: &str = ", ";

/// _Per-step._ What setup found for each assertion and measure, and the
/// state each tick folds into it.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPlan {
    /// One per series each assertion, then each measure, reads, in run-file
    /// order, an entry's instances in the catalog's order. Read by each
    /// entry's owner, never by position.
    entries: Vec<PlanEntry>,
    /// The series each window anchor names. One per selector, so entries
    /// anchored on one event share its series.
    anchors: HashMap<AssertionTarget, SeriesId>,
}

impl ResolvedPlan {
    /// Each entry with its series in `collector` and its state. `None` when
    /// `collector` isn't the one the plan was resolved against.
    pub(super) fn with_series<'p, 'c>(
        &'p self,
        collector: &'c ObservationCollector,
    ) -> Option<Vec<EntryView<'p, 'c, &'p AssertionState>>> {
        self.entries
            .iter()
            .map(|entry| entry.view(collector, &entry.state))
            .collect()
    }

    /// As [`with_series`](Self::with_series), with each state open to
    /// update.
    pub(super) fn with_series_mut<'p, 'c>(
        &'p mut self,
        collector: &'c ObservationCollector,
    ) -> Option<Vec<EntryView<'p, 'c, &'p mut AssertionState>>> {
        self.entries
            .iter_mut()
            .map(|entry| {
                let PlanEntry {
                    owner,
                    key,
                    series,
                    held,
                    state,
                } = entry;
                Some(EntryView {
                    owner: *owner,
                    key: &*key,
                    held: *held,
                    series: collector.series(*series)?,
                    state,
                })
            })
            .collect()
    }

    /// The series in `collector` the window anchor `after` names. `None`
    /// when no window is anchored on it, or `collector` isn't the plan's.
    pub(super) fn anchor<'c>(
        &self,
        after: &AssertionTarget,
        collector: &'c ObservationCollector,
    ) -> Option<&'c Series> {
        collector.series(*self.anchors.get(after)?)
    }

    /// Each entry's state, assertions then measures, in run-file order.
    #[cfg(test)]
    pub(super) fn states(&self) -> impl Iterator<Item = &AssertionState> {
        self.entries.iter().map(|entry| &entry.state)
    }
}

/// _Per-step._ Which run-file entry a plan entry evaluates: an index into
/// the run's assertions or into its measures. One enum rather than a bare
/// index, so one list's index is never read as the other's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntryOwner {
    Assertion(usize),
    Measure(usize),
}

/// One entry in the plan: whose it is, the series it is evaluated from, and
/// its state so far. An assertion on each instance of a keyed channel owns
/// one per instance.
#[derive(Debug, Clone, PartialEq)]
struct PlanEntry {
    owner: EntryOwner,
    /// The key its series is bound to.
    key: RecordedKey,
    series: SeriesId,
    /// As [`EntryView::held`].
    held: bool,
    state: AssertionState,
}

impl PlanEntry {
    fn view<'p, 'c, S>(
        &'p self,
        collector: &'c ObservationCollector,
        state: S,
    ) -> Option<EntryView<'p, 'c, S>> {
        Some(EntryView {
            owner: self.owner,
            key: &self.key,
            held: self.held,
            series: collector.series(self.series)?,
            state,
        })
    }
}

/// _Per-step._ One plan entry as a tick or finalize reads it: whose it is,
/// the key and series it reads, and its state (`S`, borrowed shared or open
/// to update).
#[derive(Debug)]
pub(super) struct EntryView<'p, 'c, S> {
    pub owner: EntryOwner,
    /// Which series: the instance, where the target names several.
    pub key: &'p RecordedKey,
    /// Whether each value holds until the next: true for a channel, whose
    /// value is what the brain could read until it was written again; false
    /// for an emission, whose values are events. A held series carries its
    /// last value into a window that starts after it.
    pub held: bool,
    pub series: &'c Series,
    pub state: S,
}

/// _Pre-step._ An assertion or measure the run can't evaluate, and every
/// reason why.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error("'{name}' on '{target}': {}", join(.problems, PROBLEM_SEPARATOR))]
pub struct UnusableEntry {
    /// Unique across assertions and measures, so it says which.
    pub(super) name: EntryName,
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
    #[error("a channel is not a node's leaf: to watch what the node writes there, use '{0}'")]
    ChannelUnderNode(String),
    /// A keyed channel written in several instances, named with no
    /// instance. The path names every one, listed, and one series is wanted.
    #[error(
        "it names several instances of a keyed channel ({}): name one with \
         instance = \"…\", or judge each with instance = \"*\"",
        .0.join(INSTANCE_SEPARATOR)
    )]
    SeveralInstances(Vec<String>),
    /// An instance the channel's writers never declared, with those they
    /// did.
    #[error(
        "its channel declares no instance '{instance}' (it declares {})",
        .declared.join(INSTANCE_SEPARATOR)
    )]
    UndeclaredInstance {
        instance: String,
        declared: Vec<String>,
    },
    /// An instance named on an emission or a channel that isn't keyed.
    #[error("it names an instance, which only a keyed channel has")]
    InstanceOnUnkeyed,
    /// A window anchored on each instance of a keyed channel: an anchor is
    /// the first stamp of one series.
    #[error(
        "it names several series ({}), where a window is anchored on one",
        .0.join(INSTANCE_SEPARATOR)
    )]
    AnchorOnSeveral(Vec<String>),
    /// A channel whose payload type has no extractor, so no value from it
    /// could be compared.
    #[error("it carries {0}, which has no extractor")]
    NoExtractor(&'static str),
    /// A nondeterministic leaf, asserted on without
    /// `allow_nondeterministic = true`.
    #[error("it is nondeterministic (set allow_nondeterministic = true to assert on it)")]
    Nondeterministic,
    /// The collector doesn't record it: the host built the collector from
    /// another run's entries.
    #[error("the run's collector doesn't record it")]
    Unrequested,
    /// `time_above` on an emission. It holds each sample until the next,
    /// which is right for a channel's state and wrong for an emission's
    /// events: a NIS doesn't stay at its value between updates.
    #[error("time_above holds each value until the next, which is wrong for an emission's events")]
    TimeAboveOnEmission,
    /// A reducer that can't reduce the kind of value the series holds.
    #[error(transparent)]
    Reduce(ReduceError),
    /// A condition that can't judge the kind of value it is given: the
    /// series', or its reducer's.
    #[error(transparent)]
    Condition(KindProblem),
    /// A series the window is anchored `after` that the run can't read, for
    /// the reason held.
    #[error("its window's anchor '{anchor}': {problem}")]
    Anchor {
        anchor: AssertionTarget,
        problem: Box<TargetProblem>,
    },
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

/// One entry to resolve, read off an assertion or a measure.
struct Wanted<'r> {
    owner: EntryOwner,
    name: &'r EntryName,
    target: &'r AssertionTarget,
    window: &'r Window,
    reduce: Option<&'r Reducer>,
    /// `None` on a measure.
    condition: Option<&'r Condition>,
    allow_nondeterministic: bool,
}

/// The plan for evaluating `assertions` and `measures`, with each one's
/// series bound in `collector`, or every one the run can't evaluate,
/// assertions first, each in run-file order.
pub(super) fn resolve(
    assertions: &[Assertion],
    measures: &[Measure],
    pipelines: &[(AgentId, &AutonomyPipeline)],
    collector: &mut ObservationCollector,
) -> Result<ResolvedPlan, Vec<UnusableEntry>> {
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

    let wanted_assertions = assertions
        .iter()
        .enumerate()
        .map(|(index, assertion)| Wanted {
            owner: EntryOwner::Assertion(index),
            name: &assertion.name,
            target: &assertion.target,
            window: &assertion.window,
            reduce: assertion.reduce.as_ref(),
            condition: Some(&assertion.condition),
            allow_nondeterministic: assertion.allow_nondeterministic,
        });
    let wanted_measures = measures.iter().enumerate().map(|(index, measure)| Wanted {
        owner: EntryOwner::Measure(index),
        name: &measure.name,
        target: &measure.target,
        window: &measure.window,
        reduce: Some(&measure.reduce),
        condition: None,
        allow_nondeterministic: measure.allow_nondeterministic,
    });

    let mut entries = Vec::with_capacity(assertions.len() + measures.len());
    let mut anchors = HashMap::new();
    let mut unusable = Vec::new();
    for wanted in wanted_assertions.chain(wanted_measures) {
        match check(&wanted, &catalogs, collector) {
            Ok(found) => {
                for key in found.targets {
                    entries.push(PlanEntry {
                        owner: wanted.owner,
                        series: collector.bind(key.clone()),
                        key,
                        held: found.held,
                        state: AssertionState::new(),
                    });
                }
                for (anchor, key) in found.anchors {
                    let series = collector.bind(key);
                    anchors.insert(anchor.clone(), series);
                }
            }
            Err(problems) => unusable.push(UnusableEntry {
                name: wanted.name.clone(),
                target: wanted.target.clone(),
                problems,
            }),
        }
    }

    if unusable.is_empty() {
        Ok(ResolvedPlan { entries, anchors })
    } else {
        Err(unusable)
    }
}

/// The keys an entry the run can evaluate reads.
struct Found<'r> {
    /// One, or one per instance under `instance = "*"`.
    targets: Vec<RecordedKey>,
    /// As [`EntryView::held`].
    held: bool,
    /// Each window anchor's selector with the key it names.
    anchors: Vec<(&'r AssertionTarget, RecordedKey)>,
}

/// The keys `wanted` reads, or every reason the run can't evaluate it:
/// its target's problems, then each anchor's.
fn check<'r>(
    wanted: &Wanted<'r>,
    catalogs: &[Catalog],
    collector: &ObservationCollector,
) -> Result<Found<'r>, Vec<TargetProblem>> {
    let mut problems = Vec::new();
    let target = match locate(
        wanted.target,
        wanted.allow_nondeterministic,
        catalogs,
        collector,
    ) {
        Ok(located) => Some(located),
        Err(mut found) => {
            problems.append(&mut found);
            None
        }
    };
    if let Some(located) = &target {
        if matches!(wanted.reduce, Some(Reducer::TimeAbove(_))) && !located.held {
            problems.push(TargetProblem::TimeAboveOnEmission);
        }
        problems.extend(kind_problem(wanted, located.kind));
    }

    let mut anchors = Vec::new();
    for anchor in wanted.window.anchors() {
        let located = locate(anchor, wanted.allow_nondeterministic, catalogs, collector).and_then(
            |located| match <[RecordedKey; 1]>::try_from(located.keys) {
                Ok([key]) => Ok(key),
                Err(keys) => Err(vec![TargetProblem::AnchorOnSeveral(instances(&keys))]),
            },
        );
        match located {
            Ok(key) => anchors.push((anchor, key)),
            Err(found) => problems.extend(found.into_iter().map(|problem| TargetProblem::Anchor {
                anchor: anchor.clone(),
                problem: Box::new(problem),
            })),
        }
    }

    match target {
        Some(located) if problems.is_empty() => Ok(Found {
            targets: located.keys,
            held: located.held,
            anchors,
        }),
        _ => Err(problems),
    }
}

/// Why `wanted`'s reducer can't reduce a series of kind `series`, or its
/// condition can't judge what it is given, if either can't: refused here
/// rather than failing on every sample once the run is under way.
fn kind_problem(wanted: &Wanted, series: ValueKind) -> Option<TargetProblem> {
    let judged = match wanted.reduce {
        Some(reducer) => match reducer.value_kind(series) {
            Ok(kind) => kind,
            Err(problem) => return Some(TargetProblem::Reduce(problem)),
        },
        None => series,
    };
    wanted
        .condition?
        .check_kind(judged)
        .err()
        .map(TargetProblem::Condition)
}

/// What `target` names in the run: its keys, every one an instance of one
/// channel or the one series, the kind of value they hold, and whether each
/// value holds until the next.
struct Located {
    keys: Vec<RecordedKey>,
    kind: ValueKind,
    /// As [`EntryView::held`].
    held: bool,
}

/// What `target` names, or every reason the run can't read it, checked
/// against the one agent the target names. A target that doesn't name a
/// series its agent records, or names the wrong instances, has no other
/// properties to check, so that problem is reported alone. Each instance
/// selected is checked, since one keyed channel's instances can have
/// different writers, and one nondeterministic writer moves the series.
fn locate(
    target: &AssertionTarget,
    allow_nondeterministic: bool,
    catalogs: &[Catalog],
    collector: &ObservationCollector,
) -> Result<Located, Vec<TargetProblem>> {
    let Some(catalog) = catalogs
        .iter()
        .find(|catalog| catalog.agent == target.agent())
    else {
        return Err(vec![TargetProblem::UnknownAgent]);
    };
    let matching = catalog.matching(target.subject());
    if matching.is_empty() {
        return Err(vec![unreported(catalog, target.subject())]);
    }
    let selected = select(target.instance(), &matching).map_err(|problem| vec![problem])?;

    let mut problems = Vec::new();
    let nondeterministic = selected
        .iter()
        .any(|(_, observable)| observable.determinism() == Determinism::Nondeterministic);
    if nondeterministic && !allow_nondeterministic {
        problems.push(TargetProblem::Nondeterministic);
    }
    // Every instance of one channel carries its key's payload type, so the
    // first selected stands for them all. A node's emission is a number.
    let kind = selected
        .first()
        .and_then(|(_, observable)| match observable.kind() {
            ObservedValueKind::Scalar => Some(ValueKind::Float),
            ObservedValueKind::Typed { type_id, type_name } => {
                let kind = collector.extracted_kind(type_id);
                if kind.is_none() {
                    problems.push(TargetProblem::NoExtractor(type_name));
                }
                kind
            }
        });
    if !collector.requests(target) {
        problems.push(TargetProblem::Unrequested);
    }

    match kind {
        Some(kind) if problems.is_empty() => Ok(Located {
            held: selected
                .iter()
                .all(|(subject, _)| matches!(subject, Subject::Channel { .. })),
            keys: selected
                .into_iter()
                .map(|(subject, _)| RecordedKey {
                    agent: catalog.agent.clone(),
                    subject: subject.clone(),
                })
                .collect(),
            kind,
        }),
        _ => Err(problems),
    }
}

/// Which of `matching`, everything a target's path names, its `instance`
/// selects: with none, the one series there is; one, by its spelling; each,
/// every one. Only a keyed channel has instances to select.
fn select<'m>(
    instance: Option<&InstanceSelector>,
    matching: &[(&'m Subject, &'m Observable)],
) -> Result<Vec<(&'m Subject, &'m Observable)>, TargetProblem> {
    let subjects: Vec<&Subject> = matching.iter().map(|(subject, _)| *subject).collect();
    let keyed = subjects
        .iter()
        .all(|subject| instance_of(subject).is_some());
    match instance {
        None if matching.len() == 1 => Ok(matching.to_vec()),
        None => Err(TargetProblem::SeveralInstances(spellings(&subjects))),
        Some(_) if !keyed => Err(TargetProblem::InstanceOnUnkeyed),
        Some(InstanceSelector::Each) => Ok(matching.to_vec()),
        Some(InstanceSelector::One(one)) => {
            let selected: Vec<(&Subject, &Observable)> = matching
                .iter()
                .copied()
                .filter(|(subject, _)| instance_of(subject) == Some(one.as_ref()))
                .collect();
            if selected.is_empty() {
                Err(TargetProblem::UndeclaredInstance {
                    instance: one.to_string(),
                    declared: spellings(&subjects),
                })
            } else {
                Ok(selected)
            }
        }
    }
}

/// The instance each of `subjects` records, spelled.
fn spellings(subjects: &[&Subject]) -> Vec<String> {
    subjects
        .iter()
        .filter_map(|subject| instance_of(subject))
        .map(str::to_owned)
        .collect()
}

/// The instance each of `keys` records, spelled.
fn instances(keys: &[RecordedKey]) -> Vec<String> {
    let subjects: Vec<&Subject> = keys.iter().map(|key| &key.subject).collect();
    spellings(&subjects)
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

    use crate::assertion::{extract::standard_extractors, AssertionValue, Bound, Condition};

    use helios_core::prelude::TfProvider;
    use helios_runtime::{
        observe::path::{channel_path, node_path},
        pipeline::autonomy_pipeline::TICK_DURATION_LEAF,
        port::{
            ChannelKey, ChannelKind, InstanceKey, Keyed, PortBus, PortDescriptor,
            PortDescriptorBuilder, Provenance,
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

    /// A value about one frame, keyed by the frame. The tests' extractor
    /// reads it as the frame's name, a string.
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

    /// The name of the entry at `position` in the lists a test resolves,
    /// counting from 1: assertions first, then measures.
    fn name_at(position: usize) -> EntryName {
        EntryName::parse(format!("e{position}")).expect("valid name")
    }

    /// An every-sample assertion on `target`. Its name is set by its
    /// position when it is resolved.
    fn assertion(target: AssertionTarget, allow_nondeterministic: bool) -> Assertion {
        Assertion {
            allow_nondeterministic,
            ..reduced(target, None)
        }
    }

    /// An assertion on `target` reduced by `reduce`.
    fn reduced(target: AssertionTarget, reduce: Option<Reducer>) -> Assertion {
        Assertion {
            name: name_at(1),
            target,
            window: Window::default(),
            reduce,
            condition: Condition::LessThan(1.0),
            allow_nondeterministic: false,
        }
    }

    /// A measure on `target` reduced by `reduce`.
    fn measure(target: AssertionTarget, reduce: Reducer) -> Measure {
        Measure {
            name: name_at(1),
            target,
            window: Window::default(),
            reduce,
            allow_nondeterministic: false,
        }
    }

    fn unusable(
        position: usize,
        target: AssertionTarget,
        problems: Vec<TargetProblem>,
    ) -> UnusableEntry {
        UnusableEntry {
            name: name_at(position),
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
        Result<ResolvedPlan, Vec<UnusableEntry>>,
    ) {
        resolve_all(assertions, &[], requested)
    }

    /// `assertions` and `measures` resolved against [`pipeline`] and a
    /// collector built for `requested`, each entry named by its position
    /// ([`name_at`]).
    fn resolve_all(
        assertions: &[Assertion],
        measures: &[Measure],
        requested: &[AssertionTarget],
    ) -> (
        ObservationCollector,
        Result<ResolvedPlan, Vec<UnusableEntry>>,
    ) {
        let assertions: Vec<Assertion> = assertions
            .iter()
            .enumerate()
            .map(|(index, assertion)| Assertion {
                name: name_at(index + 1),
                ..assertion.clone()
            })
            .collect();
        let measures: Vec<Measure> = measures
            .iter()
            .enumerate()
            .map(|(index, measure)| Measure {
                name: name_at(assertions.len() + index + 1),
                ..measure.clone()
            })
            .collect();
        let pipeline = pipeline();
        let mut extractors = standard_extractors();
        extractors.register::<FrameSample, String>(|sample| sample.0.to_string());
        let mut collector = ObservationCollector::new(requested, extractors);
        let resolved = resolve(
            &assertions,
            &measures,
            &[(AgentId::new(CAR), &pipeline)],
            &mut collector,
        );
        (collector, resolved)
    }

    /// The assertions the run can't judge, against a collector built for
    /// them.
    fn errors(assertions: &[Assertion]) -> Vec<UnusableEntry> {
        let targets: Vec<AssertionTarget> = assertions.iter().map(|a| a.target.clone()).collect();
        resolve_with(assertions, &targets)
            .1
            .err()
            .unwrap_or_default()
    }

    #[test]
    fn the_plan_names_each_entrys_owner_and_series() {
        // Two assertions on NIS share one series; SPEED has its own, read by
        // an assertion and a measure. Each starts with a fresh state of its
        // own, and a measure's index counts measures only.
        let nis = estimator(NIS);
        let speed = channel(SPEED);
        let (collector, resolved) = resolve_all(
            &[
                assertion(nis.clone(), false),
                assertion(speed.clone(), false),
                assertion(nis.clone(), true),
            ],
            &[measure(speed.clone(), Reducer::Mean)],
            &[nis.clone(), speed.clone()],
        );

        let entry = |owner, subject: Subject| PlanEntry {
            owner,
            key: key(subject.clone()),
            series: collector
                .series_id(&key(subject.clone()))
                .expect("the entry's key is bound"),
            held: matches!(subject, Subject::Channel { .. }),
            state: AssertionState::new(),
        };
        assert_eq!(
            resolved,
            Ok(ResolvedPlan {
                entries: vec![
                    entry(EntryOwner::Assertion(0), Subject::emission(ESTIMATOR, NIS)),
                    entry(EntryOwner::Assertion(1), speed_subject()),
                    entry(EntryOwner::Assertion(2), Subject::emission(ESTIMATOR, NIS)),
                    entry(EntryOwner::Measure(0), speed_subject()),
                ],
                anchors: HashMap::new(),
            })
        );
    }

    #[test]
    fn a_measure_is_checked_as_an_assertion_is() {
        // Unknown leaf, nondeterministic without permission, and a target
        // the collector doesn't record: each refused, named after the
        // assertion before it.
        let missing = estimator("missing");
        let duration = estimator(TICK_DURATION_LEAF);
        let (_, resolved) = resolve_all(
            &[assertion(estimator(NIS), false)],
            &[
                measure(missing.clone(), Reducer::Final),
                measure(duration.clone(), Reducer::Max),
                measure(channel(SPEED), Reducer::Final),
            ],
            &[estimator(NIS), missing.clone(), duration.clone()],
        );

        assert_eq!(
            resolved,
            Err(vec![
                unusable(2, missing, vec![TargetProblem::Unreported]),
                unusable(3, duration, vec![TargetProblem::Nondeterministic]),
                unusable(4, channel(SPEED), vec![TargetProblem::Unrequested]),
            ])
        );
    }

    #[test]
    fn time_above_is_refused_on_an_emission_and_accepted_on_a_channel() {
        let nis = estimator(NIS);
        let speed = channel(SPEED);
        let above = Some(Reducer::TimeAbove(1.0));
        let (_, resolved) = resolve_all(
            &[
                reduced(nis.clone(), above.clone()),
                reduced(speed.clone(), above),
            ],
            &[measure(nis.clone(), Reducer::TimeAbove(1.0))],
            &[nis.clone(), speed],
        );

        assert_eq!(
            resolved,
            Err(vec![
                unusable(1, nis.clone(), vec![TargetProblem::TimeAboveOnEmission]),
                unusable(3, nis, vec![TargetProblem::TimeAboveOnEmission]),
            ])
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
            vec![unusable(
                1,
                edges,
                vec![TargetProblem::SeveralInstances(vec![
                    "base".to_string(),
                    "wheel".to_string()
                ])]
            )]
        );
    }

    #[test]
    fn a_keyed_channel_of_one_instance_is_that_series() {
        let root = channel(ROOT);
        let (collector, resolved) = resolve_with(&[assertion_of_kind(root.clone())], &[root]);

        assert!(resolved.is_ok());
        assert!(collector.series_id(&key(frame(ROOT, "base"))).is_some());
    }

    /// The car's `channel`, at instance `instance`.
    fn frame(channel: &'static str, instance: &'static str) -> Subject {
        Subject::channel(
            keyed(channel),
            Some(InstanceKey::of::<FrameSample>(&instance)),
        )
    }

    /// The car's channel `name`, narrowed by `instance`.
    fn instance(name: &str, instance: InstanceSelector) -> AssertionTarget {
        channel(name).with_instance(instance)
    }

    fn one(instance: &str) -> InstanceSelector {
        InstanceSelector::One(instance.into())
    }

    /// An every-sample assertion on `target` whose condition fits a string
    /// series, so only the selection is under test.
    fn assertion_of_kind(target: AssertionTarget) -> Assertion {
        judging(
            target,
            None,
            Condition::NotEquals(AssertionValue::String("none".into())),
        )
    }

    /// An assertion on `target`, reduced by `reduce`, judged by `condition`.
    fn judging(
        target: AssertionTarget,
        reduce: Option<Reducer>,
        condition: Condition,
    ) -> Assertion {
        Assertion {
            condition,
            ..reduced(target, reduce)
        }
    }

    /// The key each entry of `plan` reads, in plan order.
    fn plan_keys(plan: &ResolvedPlan) -> Vec<(EntryOwner, RecordedKey)> {
        plan.entries
            .iter()
            .map(|entry| (entry.owner, entry.key.clone()))
            .collect()
    }

    #[test]
    fn an_instance_selects_one_series_of_a_keyed_channel() {
        let wheel = instance(EDGES, one("wheel"));
        let (_, resolved) = resolve_with(&[assertion_of_kind(wheel.clone())], &[wheel]);

        assert_eq!(
            plan_keys(&resolved.expect("the instance is declared")),
            [(EntryOwner::Assertion(0), key(frame(EDGES, "wheel")))]
        );
    }

    #[test]
    fn each_instance_is_an_entry_of_its_own() {
        let each = instance(EDGES, InstanceSelector::Each);
        let (collector, resolved) = resolve_all(
            &[assertion_of_kind(each.clone())],
            &[measure(each.clone(), Reducer::Count)],
            &[each],
        );

        let plan = resolved.expect("every instance resolves");
        assert_eq!(
            plan_keys(&plan),
            [
                (EntryOwner::Assertion(0), key(frame(EDGES, "base"))),
                (EntryOwner::Assertion(0), key(frame(EDGES, "wheel"))),
                (EntryOwner::Measure(0), key(frame(EDGES, "base"))),
                (EntryOwner::Measure(0), key(frame(EDGES, "wheel"))),
            ]
        );
        assert_ne!(
            collector.series_id(&key(frame(EDGES, "base"))),
            collector.series_id(&key(frame(EDGES, "wheel")))
        );
    }

    #[test]
    fn an_undeclared_instance_is_refused_listing_the_declared_ones() {
        let spare = instance(EDGES, one("spare"));

        assert_eq!(
            errors(&[assertion_of_kind(spare.clone())]),
            vec![unusable(
                1,
                spare,
                vec![TargetProblem::UndeclaredInstance {
                    instance: "spare".to_string(),
                    declared: vec!["base".to_string(), "wheel".to_string()],
                }]
            )]
        );
    }

    #[test]
    fn an_instance_is_refused_where_there_are_none() {
        let on_leaf = estimator(NIS).with_instance(InstanceSelector::Each);
        let on_unkeyed = instance(SPEED, one("front"));

        assert_eq!(
            errors(&[
                assertion(on_leaf.clone(), false),
                assertion(on_unkeyed.clone(), false),
            ]),
            vec![
                unusable(1, on_leaf, vec![TargetProblem::InstanceOnUnkeyed]),
                unusable(2, on_unkeyed, vec![TargetProblem::InstanceOnUnkeyed]),
            ]
        );
    }

    #[test]
    fn a_window_is_anchored_on_one_instance_never_each() {
        let speed = channel(SPEED);
        let each = instance(EDGES, InstanceSelector::Each);
        let wheel = instance(EDGES, one("wheel"));
        let (_, resolved) = resolve_with(
            &[
                anchored(speed.clone(), &wheel),
                anchored(speed.clone(), &each),
            ],
            &[speed.clone(), wheel, each.clone()],
        );

        assert_eq!(
            resolved,
            Err(vec![unusable(
                2,
                speed,
                vec![TargetProblem::Anchor {
                    anchor: each,
                    problem: Box::new(TargetProblem::AnchorOnSeveral(vec![
                        "base".to_string(),
                        "wheel".to_string()
                    ])),
                }]
            )])
        );
    }

    #[test]
    fn a_reducer_or_condition_that_cannot_apply_to_the_series_kind_is_refused() {
        // EDGES holds strings, NIS floats.
        let base = instance(EDGES, one("base"));
        let nis = estimator(NIS);
        let (_, resolved) = resolve_all(
            &[
                judging(base.clone(), Some(Reducer::Max), Condition::LessThan(1.0)),
                judging(base.clone(), None, Condition::LessThan(1.0)),
                judging(
                    base.clone(),
                    Some(Reducer::Count),
                    Condition::Equals(AssertionValue::String("two".into())),
                ),
                judging(nis.clone(), None, Condition::Equals(AssertionValue::Int(1))),
                judging(
                    nis.clone(),
                    Some(Reducer::Count),
                    Condition::Equals(AssertionValue::Int(0)),
                ),
            ],
            &[
                measure(base.clone(), Reducer::Mean),
                measure(base.clone(), Reducer::Final),
            ],
            &[base.clone(), nis.clone()],
        );

        let kind = |problem| vec![TargetProblem::Condition(problem)];
        assert_eq!(
            resolved,
            Err(vec![
                unusable(
                    1,
                    base.clone(),
                    vec![TargetProblem::Reduce(ReduceError::NotANumber {
                        reducer: "max",
                        value_kind: ValueKind::String,
                    })]
                ),
                unusable(
                    2,
                    base.clone(),
                    kind(KindProblem::NotANumber {
                        condition: "less_than",
                        judged: ValueKind::String,
                    })
                ),
                unusable(
                    3,
                    base.clone(),
                    kind(KindProblem::Mismatch {
                        condition: "equals",
                        judged: ValueKind::Int,
                        operand: ValueKind::String,
                    })
                ),
                unusable(
                    4,
                    nis,
                    kind(KindProblem::FloatEquality {
                        condition: "equals"
                    })
                ),
                unusable(
                    6,
                    base,
                    vec![TargetProblem::Reduce(ReduceError::NotANumber {
                        reducer: "mean",
                        value_kind: ValueKind::String,
                    })]
                ),
            ])
        );
    }

    #[test]
    fn several_instances_are_listed_in_the_message() {
        assert_eq!(
            unusable(
                1,
                channel(EDGES),
                vec![TargetProblem::SeveralInstances(vec![
                    "base".to_string(),
                    "wheel".to_string()
                ])]
            )
            .to_string(),
            "'e1' on 'agent.car.channel.edges': it names several instances of a keyed \
             channel (base, wheel): name one with instance = \"…\", or judge each with \
             instance = \"*\""
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
        // refused, and its name says which.
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
            "'e4' on 'agent.car.channel.label': it is nondeterministic \
             (set allow_nondeterministic = true to assert on it) and it carries String, \
             which has no extractor"
        );
    }

    /// `target` over a window starting `after` the series `anchor` names.
    fn anchored(target: AssertionTarget, anchor: &AssertionTarget) -> Assertion {
        Assertion {
            window: Window {
                start: Some(Bound::After {
                    after: anchor.clone(),
                    plus: 0.0,
                }),
                end: None,
            },
            ..assertion(target, false)
        }
    }

    #[test]
    fn an_anchor_is_bound_beside_its_entrys_series() {
        let nis = estimator(NIS);
        let speed = channel(SPEED);
        let (collector, resolved) =
            resolve_with(&[anchored(speed.clone(), &nis)], &[speed, nis.clone()]);

        let plan = resolved.expect("the target and its anchor resolve");
        let bound = collector
            .series_id(&key(Subject::emission(ESTIMATOR, NIS)))
            .expect("the anchor's key is bound");
        assert_eq!(plan.anchors, HashMap::from([(nis, bound)]));
    }

    #[test]
    fn an_anchor_is_checked_as_a_target_is() {
        // Unreported, nondeterministic without the entry's permission, and
        // not requested: each named as the window's anchor.
        let speed = channel(SPEED);
        let missing = estimator("missing");
        let duration = estimator(TICK_DURATION_LEAF);
        let (_, resolved) = resolve_with(
            &[
                anchored(speed.clone(), &missing),
                anchored(speed.clone(), &duration),
                anchored(speed.clone(), &estimator(NIS)),
            ],
            &[speed.clone(), missing.clone(), duration.clone()],
        );

        let anchor = |anchor: &AssertionTarget, problem| TargetProblem::Anchor {
            anchor: anchor.clone(),
            problem: Box::new(problem),
        };
        assert_eq!(
            resolved,
            Err(vec![
                unusable(
                    1,
                    speed.clone(),
                    vec![anchor(&missing, TargetProblem::Unreported)]
                ),
                unusable(
                    2,
                    speed.clone(),
                    vec![anchor(&duration, TargetProblem::Nondeterministic)]
                ),
                unusable(
                    3,
                    speed,
                    vec![anchor(&estimator(NIS), TargetProblem::Unrequested)]
                ),
            ])
        );
    }

    #[test]
    fn a_channel_holds_its_values_and_an_emission_does_not() {
        let (collector, resolved) = resolve_with(
            &[
                assertion(channel(SPEED), false),
                assertion(estimator(NIS), false),
            ],
            &[channel(SPEED), estimator(NIS)],
        );
        let plan = resolved.expect("both resolve");
        let held: Vec<bool> = plan
            .with_series(&collector)
            .expect("its own collector")
            .iter()
            .map(|entry| entry.held)
            .collect();
        assert_eq!(held, [true, false]);
    }
}
