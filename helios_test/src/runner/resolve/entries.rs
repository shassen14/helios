//! One assertion or measure checked against what the run records: its
//! target located, its reducer and condition fitted to the series' kind,
//! and each window anchor located beside it.

use super::{
    evaluations::PlannedEvaluation,
    locate::{instances, locate, locate_derived, Catalog, Located},
    plan::{EntryOwner, SeriesTarget},
    problems::TargetProblem,
};
use crate::{
    assertion::{AssertionTarget, Condition, EntryName, TargetRoot, Window},
    collector::ObservationCollector,
    report::ReportedUnit,
    statistics::within_run::Reducer,
};

use helios_runtime::observe::key::RecordedKey;

use std::collections::HashSet;

/// One entry to resolve, read off an assertion or a measure.
pub(super) struct Wanted<'r> {
    pub(super) owner: EntryOwner,
    pub(super) name: &'r EntryName,
    pub(super) target: &'r AssertionTarget,
    pub(super) window: &'r Window,
    pub(super) reduce: Option<&'r Reducer>,
    /// `None` on a measure.
    pub(super) condition: Option<&'r Condition>,
    pub(super) allow_nondeterministic: bool,
}

/// The evaluations an entry's derived target can name: those setup built,
/// and the names of those it refused.
pub(super) struct Derivable<'a> {
    pub(super) planned: &'a [PlannedEvaluation],
    pub(super) refused: &'a HashSet<&'a EntryName>,
}

/// The series an entry the run can evaluate reads.
pub(super) struct Found<'r> {
    /// One, or one per instance under `instance = "*"`.
    pub(super) targets: Vec<SeriesTarget>,
    /// As [`EntryView::held`].
    pub(super) held: bool,
    /// As [`EntryView::unit`].
    pub(super) unit: Option<ReportedUnit>,
    /// Each window anchor's selector with the key it names.
    pub(super) anchors: Vec<(&'r AssertionTarget, RecordedKey)>,
}

/// The keys `wanted` reads, or every reason the run can't evaluate it:
/// its target's problems, then each anchor's.
pub(super) fn check<'r>(
    wanted: &Wanted<'r>,
    catalogs: &[Catalog],
    derived: &Derivable,
    collector: &ObservationCollector,
) -> Result<Found<'r>, Vec<TargetProblem>> {
    let mut problems = Vec::new();
    let located = match wanted.target.root() {
        TargetRoot::Derived(name) => {
            locate_derived(wanted.target, name, wanted.allow_nondeterministic, derived)
        }
        TargetRoot::Agent { .. } => locate(
            wanted.target,
            wanted.allow_nondeterministic,
            catalogs,
            collector,
        ),
    };
    let target = match located {
        Ok(located) => Some(located),
        Err(mut found) => {
            problems.append(&mut found);
            None
        }
    };
    let mut unit = None;
    if let Some(located) = &target {
        if matches!(wanted.reduce, Some(Reducer::TimeAbove(_))) && !located.held {
            problems.push(TargetProblem::TimeAboveOnEmission);
        }
        match judged_unit(wanted, located) {
            Ok(judged) => unit = judged,
            Err(problem) => problems.push(problem),
        }
    }

    let mut anchors = Vec::new();
    for anchor in wanted.window.anchors() {
        // Load refuses an anchor on a derived series, so each is recorded.
        let located = locate(anchor, wanted.allow_nondeterministic, catalogs, collector).and_then(
            |located| match <[SeriesTarget; 1]>::try_from(located.series) {
                Ok([SeriesTarget::Recorded(key)]) => Ok(key),
                Ok([SeriesTarget::Derived { .. }]) => Err(vec![TargetProblem::Unreported]),
                Err(series) => Err(vec![TargetProblem::AnchorOnSeveral(instances(&series))]),
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
            targets: located.series,
            held: located.held,
            unit,
            anchors,
        }),
        _ => Err(problems),
    }
}

/// The unit of the value `wanted` judges or measures in the series
/// `located`, or why its reducer can't reduce that series' kind or its
/// condition can't judge what it is given: refused here rather than failing
/// on every sample once the run is under way.
fn judged_unit(wanted: &Wanted, located: &Located) -> Result<Option<ReportedUnit>, TargetProblem> {
    let (kind, si_unit) = match wanted.reduce {
        Some(reducer) => (
            reducer
                .value_kind(located.kind)
                .map_err(TargetProblem::Reduce)?,
            reducer.si_unit(located.si_unit),
        ),
        None => (located.kind, located.si_unit),
    };
    if let Some(condition) = wanted.condition {
        condition
            .check_kind(kind)
            .map_err(TargetProblem::Condition)?;
    }
    Ok(ReportedUnit::of(kind, si_unit))
}

#[cfg(test)]
mod tests {
    use crate::{
        assertion::{
            condition::KindProblem, AssertionTarget, AssertionValue, Condition, InstanceSelector,
            ValueKind,
        },
        runner::resolve::{
            fixtures::{
                anchored, assertion, channel, estimator, instance, judging, measure, one, reduced,
                resolve_all, resolve_with, unusable, EDGES, NIS, SPEED,
            },
            problems::TargetProblem,
        },
        statistics::within_run::{ReduceError, Reducer},
    };

    use helios_runtime::pipeline::autonomy_pipeline::TICK_DURATION_LEAF;

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
}
