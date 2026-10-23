//! What a target names in the run: the agent's [`Catalog`] of what it
//! records, the series a recorded or derived target selects there, the
//! kind and unit of value they hold, and why a target names none.

use super::{entries::Derivable, plan::SeriesTarget, problems::TargetProblem};
use crate::{
    assertion::{
        extract::Extracted, AssertionTarget, InstanceSelector, TargetRoot, TargetSubject, ValueKind,
    },
    collector::{instance_of, ObservationCollector},
    evaluation::path::DerivedName,
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::AgentId;
use helios_runtime::{
    observe::{
        key::{RecordedKey, Subject},
        path::channel_path,
    },
    pipeline::{FrameConventions, FramePlacement},
    port::{ChannelDescription, ChannelKey, Determinism, Observable, ObservableValue},
};

/// What one agent reports, listed once for every assertion to search.
pub(super) struct Catalog<'a> {
    pub agent: &'a AgentId,
    /// Each subject the agent's pipeline records, with what it records.
    pub observables: Vec<(Subject, Observable)>,
    /// Each channel a node writes, with the node's name.
    pub(super) node_written: Vec<(&'a str, &'a ChannelKey)>,
    /// Each channel's description, one per writer.
    pub descriptions: &'a [ChannelDescription],
    /// Each owned frame's axis convention, as its owner gives it.
    pub frame_conventions: &'a FrameConventions,
    /// Where each frame a node defines sits in the world.
    pub frame_placements: &'a [FramePlacement],
}

impl Catalog<'_> {
    /// Every entry `subject`, a target's subject, names: at most one
    /// emission, or every instance of a channel. A target spells a channel
    /// by name alone, which is enough: the build refuses one name for two
    /// channels of an agent.
    pub fn matching(&self, subject: &TargetSubject) -> Vec<(&Subject, &Observable)> {
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

/// What `target` names in the run: its series, every one an instance of one
/// channel or the one series, the kind of value they hold and its unit, and
/// whether each value holds until the next.
pub(super) struct Located {
    pub(super) series: Vec<SeriesTarget>,
    pub(super) kind: ValueKind,
    /// The leaf's, or the extractor's; `None` where undeclared.
    pub(super) si_unit: Option<SiUnit>,
    /// As [`EntryView::held`].
    pub(super) held: bool,
}

/// What `target` names, or every reason the run can't read it, checked
/// against the one agent the target names. A target that doesn't name a
/// series its agent records, or names the wrong instances, has no other
/// properties to check, so that problem is reported alone. Each instance
/// selected is checked, since one keyed channel's instances can have
/// different writers, and one nondeterministic writer moves the series.
pub(super) fn locate(
    target: &AssertionTarget,
    allow_nondeterministic: bool,
    catalogs: &[Catalog],
    collector: &ObservationCollector,
) -> Result<Located, Vec<TargetProblem>> {
    let (catalog, selected) = find(target, catalogs)?;

    let mut problems = Vec::new();
    let nondeterministic = selected
        .iter()
        .any(|(_, observable)| observable.determinism() == Determinism::Nondeterministic);
    if nondeterministic && !allow_nondeterministic {
        problems.push(TargetProblem::Nondeterministic);
    }
    // Every instance of one channel carries its key's payload type, so the
    // first selected stands for them all. A node's emission is a number in
    // the unit its leaf declares.
    let extracted = selected
        .first()
        .and_then(|(_, observable)| match observable.value() {
            ObservableValue::Scalar { si_unit } => Some(Extracted {
                kind: ValueKind::Float,
                si_unit: Some(si_unit),
            }),
            ObservableValue::Typed { type_id, type_name } => {
                let extracted = collector.extracted(type_id);
                if extracted.is_none() {
                    problems.push(TargetProblem::NoExtractor(type_name));
                }
                extracted
            }
        });
    match extracted {
        Some(Extracted { kind, si_unit }) if problems.is_empty() => Ok(Located {
            held: selected
                .iter()
                .all(|(subject, _)| matches!(subject, Subject::Channel { .. })),
            series: selected
                .into_iter()
                .map(|(subject, _)| {
                    SeriesTarget::Recorded(RecordedKey {
                        agent: catalog.agent.clone(),
                        subject: subject.clone(),
                    })
                })
                .collect(),
            kind,
            si_unit,
        }),
        _ => Err(problems),
    }
}

/// An agent's catalog, and the series a target selects in it, each with
/// what it records.
pub(super) type Selected<'c, 'a> = (&'c Catalog<'a>, Vec<(&'c Subject, &'c Observable)>);

/// The catalog of the agent `target` names and the series it selects
/// there, or why it names none. A target that doesn't name a series its
/// agent records, or names the wrong instances, has no other properties to
/// check, so that problem is reported alone.
pub(super) fn find<'c, 'a>(
    target: &AssertionTarget,
    catalogs: &'c [Catalog<'a>],
) -> Result<Selected<'c, 'a>, Vec<TargetProblem>> {
    let TargetRoot::Agent { agent, subject } = target.root() else {
        return Err(vec![TargetProblem::Unreported]);
    };
    let Some(catalog) = catalogs.iter().find(|catalog| catalog.agent == agent) else {
        return Err(vec![TargetProblem::UnknownAgent]);
    };
    let matching = catalog.matching(subject);
    if matching.is_empty() {
        return Err(vec![unreported(catalog, subject)]);
    }
    let selected = select(target.instance(), &matching).map_err(|problem| vec![problem])?;
    Ok((catalog, selected))
}

/// What the derived `target`, metric `name` of an evaluation, names: one
/// series of floats in the metric's unit, held when the evaluation's base
/// is, or why the run can't read it. A refused evaluation's metric is
/// refused for that alone; its own reasons are reported with it.
pub(super) fn locate_derived(
    target: &AssertionTarget,
    name: &DerivedName,
    allow_nondeterministic: bool,
    derivable: &Derivable,
) -> Result<Located, Vec<TargetProblem>> {
    if derivable.refused.contains(name.evaluation()) {
        return Err(vec![TargetProblem::EvaluationUnusable]);
    }
    // Load checked the evaluation exists, and the metric when its kind's
    // menu is fixed.
    let Some((evaluation, planned)) = derivable
        .planned
        .iter()
        .enumerate()
        .find(|(_, planned)| &planned.name == name.evaluation())
    else {
        return Err(vec![TargetProblem::Unreported]);
    };
    let Some(metric) = planned
        .metrics
        .iter()
        .position(|metric| metric.name == name.metric())
    else {
        return Err(vec![TargetProblem::UnofferedMetric(
            planned.metrics.iter().map(|metric| metric.name).collect(),
        )]);
    };

    let mut problems = Vec::new();
    if target.instance().is_some() {
        problems.push(TargetProblem::InstanceOnDerived);
    }
    if planned.nondeterministic && !allow_nondeterministic {
        problems.push(TargetProblem::Nondeterministic);
    }
    if !problems.is_empty() {
        return Err(problems);
    }
    Ok(Located {
        series: vec![SeriesTarget::Derived { evaluation, metric }],
        kind: ValueKind::Float,
        si_unit: Some(planned.metrics[metric].si_unit),
        held: planned.held,
    })
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
pub(super) fn spellings(subjects: &[&Subject]) -> Vec<String> {
    subjects
        .iter()
        .filter_map(|subject| instance_of(subject))
        .map(str::to_owned)
        .collect()
}

/// The instance each of `series` records, spelled.
pub(super) fn instances(series: &[SeriesTarget]) -> Vec<String> {
    let subjects: Vec<&Subject> = series
        .iter()
        .filter_map(|series| match series {
            SeriesTarget::Recorded(key) => Some(&key.subject),
            SeriesTarget::Derived { .. } => None,
        })
        .collect();
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

#[cfg(test)]
mod tests {
    use crate::{
        assertion::InstanceSelector,
        runner::resolve::{
            fixtures::{
                assertion, assertion_of_kind, channel, errors, estimator, frame, instance, key,
                one, plan_keys, resolve_with, target, unusable, CAR, EDGES, ESTIMATOR, GOAL, LABEL,
                NIS, ODOMETRY, PLANNER, ROOT, SHARED, SPEED,
            },
            plan::EntryOwner,
            problems::TargetProblem,
        },
    };

    use helios_runtime::pipeline::autonomy_pipeline::TICK_DURATION_LEAF;

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
        let (collector, resolved) = resolve_with(&[assertion_of_kind(root.clone())]);

        assert!(resolved.is_ok());
        assert!(collector.series_id(&key(frame(ROOT, "base"))).is_some());
    }

    #[test]
    fn an_instance_selects_one_series_of_a_keyed_channel() {
        let wheel = instance(EDGES, one("wheel"));
        let (_, resolved) = resolve_with(&[assertion_of_kind(wheel.clone())]);

        assert_eq!(
            plan_keys(&resolved.expect("the instance is declared")),
            [(EntryOwner::Assertion(0), key(frame(EDGES, "wheel")))]
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
}
