//! [`Reached`]: what a finished run reached, read by each part of its
//! report: every entry's line, what the recording dropped, and each
//! evaluation's line.

use super::{
    entries::{instance, report_evaluation, report_measure, spelled, unread},
    verdict::judge,
    windowed::Windowed,
};
use crate::{
    assertion::Window,
    collector::ObservationCollector,
    evaluation::derived::Derived,
    report::{AssertionReportEntry, EvaluationReportEntry, MeasureReportEntry, Verdict},
    runner::{
        resolve::{evaluations::PlannedEvaluation, EntryOwner, EntryView, ResolvedPlan},
        state::AssertionState,
        window::{first_stamp, place, Placement},
        Phase, Runner, TickSpan, FOREIGN_COLLECTOR, NEVER_SET_UP,
    },
};

/// _Post-step._ What the run reached: its plan, where it was set up, each
/// entry with the series it reads, each evaluation's derived series, and
/// whether its clock reached the time budget.
pub(super) struct Reached<'p, 'c> {
    runner: &'p Runner,
    plan: Option<&'p ResolvedPlan>,
    collector: &'c ObservationCollector,
    /// Empty when the run isn't ready to judge.
    judged: Vec<EntryView<'p, 'c, &'p AssertionState>>,
    derived: Option<&'c [Derived]>,
    ran_out: bool,
}

impl<'p, 'c> Reached<'p, 'c> {
    /// What `runner` reached, its entries read from `collector` and the
    /// series its evaluations `derived`. With it, why the run fails though
    /// its host stopped it: a host that finalizes without having set the
    /// run up, or with another run's collector, fails it here as `tick`
    /// would have.
    pub(super) fn new(
        runner: &'p Runner,
        collector: &'c ObservationCollector,
        derived: Option<&'c [Derived]>,
    ) -> (Self, Option<&'static str>) {
        let (plan, judged, failure) = match &runner.phase {
            Phase::Ready(plan) => {
                match derived.and_then(|derived| plan.with_series(collector, derived)) {
                    Some(judged) => (Some(plan), judged, None),
                    None => (Some(plan), Vec::new(), Some(FOREIGN_COLLECTOR)),
                }
            }
            Phase::AwaitingPipelines => (None, Vec::new(), Some(NEVER_SET_UP)),
            Phase::Aborted => (None, Vec::new(), None),
        };
        let ran_out = runner.ticks.is_some_and(|span| {
            runner
                .run
                .termination()
                .budget_spent(span.first, span.latest)
        });
        let reached = Self {
            runner,
            plan,
            collector,
            judged,
            derived,
            ran_out,
        };
        (reached, failure)
    }

    /// One line per assertion on each series it read, in run-file order.
    pub(super) fn assertions(&self) -> Vec<AssertionReportEntry> {
        self.runner
            .run
            .assertions()
            .iter()
            .enumerate()
            .flat_map(|(index, assertion)| {
                self.each_or_unread(EntryOwner::Assertion(index))
                    .into_iter()
                    .map(|entry| {
                        let (series, verdict) = match entry.zip(self.runner.ticks) {
                            Some((entry, span)) => {
                                let windowed = Windowed::new(
                                    entry,
                                    &assertion.window,
                                    self.place_in(&assertion.window, span),
                                    span,
                                );
                                (
                                    windowed.summary(&assertion.reduce),
                                    judge(assertion, entry.rejection(), &windowed),
                                )
                            }
                            None => (unread(), Verdict::Unjudged { actual: None }),
                        };
                        AssertionReportEntry::new(
                            assertion.name.to_string(),
                            assertion.target.clone(),
                            entry
                                .and_then(|entry| entry.agent)
                                .map(|agent| agent.as_str().to_owned()),
                            entry.and_then(|entry| entry.key.and_then(instance)),
                            assertion.window.clone(),
                            assertion.reduce.clone(),
                            entry.and_then(|entry| entry.unit),
                            assertion.condition.clone(),
                            series,
                            verdict,
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .collect()
    }

    /// What the recording dropped: values under no bound series, then each
    /// evaluation's refusals, then each value a measure's series turned
    /// away. An input value an evaluation refused, a sample it couldn't
    /// compute, or a metric's value it couldn't give leaves its metrics
    /// graded on fewer values than the run recorded. Only a run that was set
    /// up reports them; in one aborted at setup, a refused entry's key is
    /// unbound by design.
    pub(super) fn recording(&self) -> Vec<String> {
        let mut recording: Vec<String> = match &self.runner.phase {
            Phase::Ready(_) => self
                .collector
                .unbound_arrivals()
                .iter()
                .map(ToString::to_string)
                .collect(),
            Phase::AwaitingPipelines | Phase::Aborted => Vec::new(),
        };
        for (index, evaluation) in self.planned().iter().enumerate() {
            let refused = self.collector.refused(evaluation.id).unwrap_or_default();
            for (input, refused) in evaluation.inputs.iter().zip(refused) {
                if let Some(first) = refused.first {
                    recording.push(format!(
                        "evaluation '{}' input '{}': {} value(s) refused, the first {first}",
                        evaluation.name, input.role, refused.count
                    ));
                }
            }
            let Some(derived) = self.derived.and_then(|derived| derived.get(index)) else {
                continue;
            };
            if let Some(first) = derived.not_computed.first {
                recording.push(format!(
                    "evaluation '{}': {} sample(s) not computed, the first {first}",
                    evaluation.name, derived.not_computed.count
                ));
            }
            for (metric, refused) in evaluation.metrics.iter().zip(&derived.values_refused) {
                if let Some(first) = refused.first {
                    recording.push(format!(
                        "evaluation '{}' metric '{}': {} value(s) refused, the first {first}",
                        evaluation.name, metric.name, refused.count
                    ));
                }
            }
        }
        recording.extend(self.rejected());
        recording
    }

    /// One line per measure on each series it read, in run-file order.
    pub(super) fn measures(&self) -> Vec<MeasureReportEntry> {
        self.runner
            .run
            .measures()
            .iter()
            .enumerate()
            .flat_map(|(index, measure)| {
                self.each_or_unread(EntryOwner::Measure(index))
                    .into_iter()
                    .map(|judged| {
                        let windowed = judged.zip(self.runner.ticks).map(|(entry, span)| {
                            Windowed::new(
                                entry,
                                &measure.window,
                                self.place_in(&measure.window, span),
                                span,
                            )
                        });
                        report_measure(measure, judged, windowed.as_ref())
                    })
                    .collect::<Vec<_>>()
            })
            .collect()
    }

    /// One line per evaluation that derived its series, in run-file order.
    pub(super) fn evaluations(&self) -> Vec<EvaluationReportEntry> {
        self.planned()
            .iter()
            .zip(self.derived.into_iter().flatten())
            .map(|(evaluation, derived)| {
                report_evaluation(
                    evaluation,
                    derived,
                    self.collector.refused(evaluation.id).unwrap_or_default(),
                )
            })
            .collect()
    }

    /// Each value a measure's recorded series turned away, in run-file
    /// order. A measure's value never fails the run, but a value its series
    /// turned away is a fault in the recording, like an unbound arrival. A
    /// derived series' rejection is its evaluation's, reported with it.
    fn rejected(&self) -> Vec<String> {
        self.runner
            .run
            .measures()
            .iter()
            .enumerate()
            .flat_map(|(index, measure)| {
                self.each_or_unread(EntryOwner::Measure(index))
                    .into_iter()
                    .flatten()
                    .filter(|entry| !entry.is_derived())
                    .filter_map(|entry| {
                        let (rejection, key) = entry.rejection().zip(entry.key)?;
                        Some(format!(
                            "measure '{}' on '{}': value rejected: {rejection}",
                            measure.name,
                            spelled(key)
                        ))
                    })
                    .collect::<Vec<_>>()
            })
            .collect()
    }

    /// The entries `owner` owns, one per series it reads, or one with none
    /// when the run never read it: an entry the run never read still gets
    /// its line.
    fn each_or_unread(
        &self,
        owner: EntryOwner,
    ) -> Vec<Option<&EntryView<'p, 'c, &'p AssertionState>>> {
        let entries: Vec<_> = self
            .judged
            .iter()
            .filter(|entry| entry.owner == owner)
            .collect();
        if entries.is_empty() {
            vec![None]
        } else {
            entries.into_iter().map(Some).collect()
        }
    }

    /// Where `window` falls in the run, each anchor at its series' first
    /// stamp in the run.
    fn place_in(&self, window: &Window, span: TickSpan) -> Placement {
        place(window, span, self.ran_out, |after| {
            let series = self.plan?.anchor(after, self.collector)?;
            first_stamp(series.samples(), span.latest)
        })
    }

    /// Each evaluation the plan holds; none in a run never set up.
    fn planned(&self) -> &'p [PlannedEvaluation] {
        self.plan.map_or(&[][..], |plan| plan.evaluations())
    }
}
