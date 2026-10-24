//! [`ResolvedPlan`]: what setup found for each assertion and measure,
//! the series each reads, bound in the collector, and the state each
//! carries; and [`resolve`], which builds it once the pipelines exist, so a
//! run that could never pass stops before its first tick and names every
//! problem at once.

use super::{
    entries::{check, Derivable, Wanted},
    evaluations::{resolve_evaluations, PlannedEvaluation, RunEvaluations},
    locate::Catalogs,
    problems::{Unusable, UnusableEntry},
};
use crate::{
    assertion::{Assertion, AssertionTarget, Measure, TargetRoot},
    collector::{ObservationCollector, Rejection, Series, SeriesId},
    evaluation::{derived::Derived, path::EvaluationName, EvaluatorRegistry},
    report::ReportedUnit,
    runner::state::AssertionState,
};

use helios_core::prelude::{AgentId, MonotonicTime};
use helios_runtime::{observe::key::RecordedKey, AutonomyPipeline};

use std::collections::HashMap;

/// _Per-step._ What setup found for each assertion and measure, and the
/// state each tick folds into it.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPlan {
    /// One per series each assertion, then each measure, reads, in run-file
    /// order, an entry's instances in the catalog's order. Read by each
    /// entry's owner, never by position.
    pub(super) entries: Vec<PlanEntry>,
    /// The series each window anchor names. One per selector, so entries
    /// anchored on one event share its series.
    anchors: HashMap<AssertionTarget, SeriesId>,
    /// Each evaluation, in run-file order.
    evaluations: Vec<PlannedEvaluation>,
}

impl ResolvedPlan {
    /// Each evaluation, in run-file order.
    pub(in crate::runner) fn evaluations(&self) -> &[PlannedEvaluation] {
        &self.evaluations
    }

    /// What each evaluation derived from the values stamped at or before
    /// `until`, in run-file order: only the metrics an entry reads. `None`
    /// when `collector` isn't the one the plan was resolved against.
    pub(in crate::runner) fn derive(
        &self,
        collector: &ObservationCollector,
        until: MonotonicTime,
    ) -> Option<Vec<Derived>> {
        self.evaluations
            .iter()
            .enumerate()
            .map(|(index, evaluation)| {
                let wanted: Vec<usize> = self
                    .entries
                    .iter()
                    .filter_map(|entry| match entry.source {
                        SeriesSource::Derived { evaluation, metric } if evaluation == index => {
                            Some(metric)
                        }
                        _ => None,
                    })
                    .collect();
                collector.derive(evaluation.id, until, &wanted)
            })
            .collect()
    }

    /// Each entry with its series and its state: a recorded series from
    /// `collector`, a derived one from `derived`, what
    /// [`derive`](Self::derive) gave. `None` when `collector` isn't the one
    /// the plan was resolved against, or `derived` misses an evaluation.
    pub(in crate::runner) fn with_series<'p, 'c>(
        &'p self,
        collector: &'c ObservationCollector,
        derived: &'c [Derived],
    ) -> Option<Vec<EntryView<'p, 'c, &'p AssertionState>>> {
        self.entries
            .iter()
            .map(|entry| {
                let series = match &entry.source {
                    SeriesSource::Recorded { series, .. } => collector.series(*series)?,
                    SeriesSource::Derived { evaluation, metric } => {
                        derived.get(*evaluation)?.metrics.get(*metric)?.as_ref()?
                    }
                };
                Some(entry.view(series, &entry.state))
            })
            .collect()
    }

    /// Each entry on a recorded series, with its series in `collector` and
    /// its state open to update: what a tick judges. A derived series
    /// doesn't exist until finalize. `None` when `collector` isn't the one
    /// the plan was resolved against.
    pub(in crate::runner) fn with_series_mut<'p, 'c>(
        &'p mut self,
        collector: &'c ObservationCollector,
    ) -> Option<Vec<EntryView<'p, 'c, &'p mut AssertionState>>> {
        self.entries
            .iter_mut()
            .filter_map(|entry| {
                let PlanEntry {
                    owner,
                    source,
                    agent,
                    held,
                    unit,
                    state,
                } = entry;
                let SeriesSource::Recorded { key, series } = source else {
                    return None;
                };
                Some(collector.series(*series).map(|series| EntryView {
                    owner: *owner,
                    key: Some(&*key),
                    agent: agent.as_ref(),
                    held: *held,
                    unit: *unit,
                    series,
                    state,
                }))
            })
            .collect()
    }

    /// The series in `collector` the window anchor `after` names. `None`
    /// when no window is anchored on it, or `collector` isn't the plan's.
    pub(in crate::runner) fn anchor<'c>(
        &self,
        after: &AssertionTarget,
        collector: &'c ObservationCollector,
    ) -> Option<&'c Series> {
        collector.series(*self.anchors.get(after)?)
    }

    /// Each entry's state, assertions then measures, in run-file order.
    #[cfg(test)]
    pub(in crate::runner) fn states(&self) -> impl Iterator<Item = &AssertionState> {
        self.entries.iter().map(|entry| &entry.state)
    }

    /// Its evaluations, for a run stopped before it derived them.
    pub(in crate::runner) fn into_evaluations(self) -> Vec<PlannedEvaluation> {
        self.evaluations
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
/// one per instance, and one on every agent's evaluation one per agent.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PlanEntry {
    pub(super) owner: EntryOwner,
    pub(super) source: SeriesSource,
    /// As [`EntryView::agent`].
    pub(super) agent: Option<AgentId>,
    /// As [`EntryView::held`].
    pub(super) held: bool,
    /// As [`EntryView::unit`].
    pub(super) unit: Option<ReportedUnit>,
    pub(super) state: AssertionState,
}

impl PlanEntry {
    fn view<'p, 'c, S>(&'p self, series: &'c Series, state: S) -> EntryView<'p, 'c, S> {
        EntryView {
            owner: self.owner,
            key: match &self.source {
                SeriesSource::Recorded { key, .. } => Some(key),
                SeriesSource::Derived { .. } => None,
            },
            agent: self.agent.as_ref(),
            held: self.held,
            unit: self.unit,
            series,
            state,
        }
    }
}

/// Where an entry's series comes from.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum SeriesSource {
    /// Recorded under `key`, bound in the collector as `series`.
    Recorded { key: RecordedKey, series: SeriesId },
    /// Metric `metric` of the plan's evaluation `evaluation`, both indices.
    Derived { evaluation: usize, metric: usize },
}

/// One series an entry reads, as setup found it.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum SeriesTarget {
    Recorded(RecordedKey),
    /// As [`SeriesSource::Derived`].
    Derived {
        evaluation: usize,
        metric: usize,
    },
}

/// _Per-step._ One plan entry as a tick or finalize reads it: whose it is,
/// the key and series it reads, and its state (`S`, borrowed shared or open
/// to update).
#[derive(Debug)]
pub(in crate::runner) struct EntryView<'p, 'c, S> {
    pub owner: EntryOwner,
    /// Which recorded series: the instance, where the target names
    /// several. `None` for a derived series, which its target names whole.
    pub key: Option<&'p RecordedKey>,
    /// The agent whose evaluation it reads, when its target selects every
    /// agent's (`eval.agent.*`); `None` for every other entry, whose target
    /// names its agent.
    pub agent: Option<&'p AgentId>,
    /// Whether each value holds until the next: true for a channel, whose
    /// value is what the brain could read until it was written again; false
    /// for an emission, whose values are events. A held series carries its
    /// last value into a window that starts after it.
    pub held: bool,
    /// The unit the entry's value is read in, `None` for a bool or string.
    pub unit: Option<ReportedUnit>,
    pub series: &'c Series,
    pub state: S,
}

impl<S: std::ops::Deref<Target = AssertionState>> EntryView<'_, '_, S> {
    /// Whether its series is derived, so computed whole at finalize rather
    /// than reached tick by tick.
    pub fn is_derived(&self) -> bool {
        self.key.is_none()
    }

    /// Why a value of its series was turned away, if one was: a recorded
    /// series' once the run reached it, a derived series' as it was
    /// computed.
    pub fn rejection(&self) -> Option<Rejection> {
        if self.is_derived() {
            self.series.rejection()
        } else {
            self.state.rejection()
        }
    }
}

/// _Pre-step._ A run setup refused: the evaluations it could still bind,
/// which the report names though the run never derives them, and every
/// one, assertion and measure it can't use.
#[derive(Debug, Clone, PartialEq)]
pub(in crate::runner) struct Unresolved {
    pub bound: Vec<PlannedEvaluation>,
    pub unusable: Unusable,
}

/// The plan for evaluating `evaluations` (each suite's on each agent it is
/// applied to), `assertions` and `measures`, with each one's series and
/// inputs bound in `collector`, or what the run could bind of them beside
/// every one it can't evaluate, each list in run-file order. Evaluations are resolved first, since an entry
/// may read one's metric. One on an `unbuilt` agent is refused for that.
pub(in crate::runner) fn resolve(
    evaluations: RunEvaluations,
    assertions: &[Assertion],
    measures: &[Measure],
    registry: &EvaluatorRegistry,
    pipelines: &[(AgentId, &AutonomyPipeline)],
    unbuilt: &[AgentId],
    collector: &mut ObservationCollector,
) -> Result<ResolvedPlan, Unresolved> {
    let catalogs = Catalogs::new(pipelines, unbuilt);

    let mut unusable = Unusable::default();
    let (planned, refused) = resolve_evaluations(evaluations, registry, &catalogs, collector);
    unusable.evaluations = refused;
    let refused: Vec<&EvaluationName> = unusable
        .evaluations
        .iter()
        .map(|evaluation| &evaluation.name)
        .collect();
    let derived = Derivable {
        planned: &planned,
        refused: &refused,
    };

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
    for wanted in wanted_assertions.chain(wanted_measures) {
        let each_agent = matches!(
            wanted.target.root(),
            TargetRoot::Derived(derived) if derived.selects_each_agent()
        );
        match check(&wanted, &catalogs, &derived, collector) {
            Ok(found) => {
                for target in found.targets {
                    let (source, agent) = match target {
                        SeriesTarget::Recorded(key) => (
                            SeriesSource::Recorded {
                                series: collector.bind(wanted.target, key.clone()),
                                key,
                            },
                            None,
                        ),
                        SeriesTarget::Derived { evaluation, metric } => (
                            SeriesSource::Derived { evaluation, metric },
                            planned
                                .get(evaluation)
                                .and_then(|planned| planned.name.applied())
                                .filter(|_| each_agent)
                                .map(|(agent, _)| agent.clone()),
                        ),
                    };
                    entries.push(PlanEntry {
                        owner: wanted.owner,
                        source,
                        agent,
                        held: found.held,
                        unit: found.unit,
                        state: AssertionState::new(),
                    });
                }
                for (anchor, key) in found.anchors {
                    let series = collector.bind(anchor, key);
                    anchors.insert(anchor.clone(), series);
                }
            }
            Err(problems) => unusable.entries.push(UnusableEntry {
                name: wanted.name.clone(),
                target: wanted.target.clone(),
                problems,
            }),
        }
    }

    if unusable.is_empty() {
        Ok(ResolvedPlan {
            entries,
            anchors,
            evaluations: planned,
        })
    } else {
        Err(Unresolved {
            bound: planned,
            unusable,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::InstanceSelector,
        report::ReportedUnit,
        runner::{
            resolve::{
                fixtures::{
                    anchored, assertion, assertion_of_kind, channel, errors, estimator, frame,
                    instance, key, measure, plan_keys, resolve_all, resolve_with, speed_subject,
                    unusable, EDGES, ESTIMATOR, LABEL, NIS, SPEED,
                },
                problems::TargetProblem,
            },
            state::AssertionState,
        },
        statistics::within_run::Reducer,
    };

    use helios_core::kernel::si_unit::SiUnit;
    use helios_runtime::{
        observe::key::Subject, pipeline::autonomy_pipeline::TICK_DURATION_LEAF, port::ChannelKey,
    };

    use std::collections::HashMap;

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
        );

        let entry = |owner, subject: Subject| PlanEntry {
            owner,
            source: SeriesSource::Recorded {
                key: key(subject.clone()),
                series: collector
                    .series_id(&key(subject.clone()))
                    .expect("the entry's key is bound"),
            },
            agent: None,
            held: matches!(subject, Subject::Channel { .. }),
            // NIS declares itself dimensionless; a bare f64 channel declares
            // nothing.
            unit: Some(match subject {
                Subject::Channel { .. } => ReportedUnit::Undeclared,
                _ => ReportedUnit::Declared(SiUnit::Dimensionless),
            }),
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
                evaluations: vec![],
            })
        );
    }

    #[test]
    fn a_channel_target_binds_the_key_its_agent_declares_payload_type_included() {
        // The path spells only the name; the bound key is the catalog's, so
        // a value of another payload type under that name lands nowhere.
        let (collector, resolved) = resolve_with(&[assertion(channel(SPEED), false)]);

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
        let (collector, resolved) = resolve_with(&[assertion(label.clone(), false)]);

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
    fn each_instance_is_an_entry_of_its_own() {
        let each = instance(EDGES, InstanceSelector::Each);
        let (collector, resolved) = resolve_all(
            &[assertion_of_kind(each.clone())],
            &[measure(each.clone(), Reducer::Count)],
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
    fn a_channel_holds_its_values_and_an_emission_does_not() {
        let (collector, resolved) = resolve_with(&[
            assertion(channel(SPEED), false),
            assertion(estimator(NIS), false),
        ]);
        let plan = resolved.expect("both resolve");
        let held: Vec<bool> = plan
            .with_series(&collector, &[])
            .expect("its own collector")
            .iter()
            .map(|entry| entry.held)
            .collect();
        assert_eq!(held, [true, false]);
    }

    #[test]
    fn an_anchor_is_bound_beside_its_entrys_series() {
        let nis = estimator(NIS);
        let speed = channel(SPEED);
        let (collector, resolved) = resolve_with(&[anchored(speed.clone(), &nis)]);

        let plan = resolved.expect("the target and its anchor resolve");
        let bound = collector
            .series_id(&key(Subject::emission(ESTIMATOR, NIS)))
            .expect("the anchor's key is bound");
        assert_eq!(plan.anchors, HashMap::from([(nis, bound)]));
    }
}
