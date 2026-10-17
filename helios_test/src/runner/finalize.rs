use super::{
    resolve::{EntryOwner, EntryView},
    state::AssertionState,
    window::{first_stamp, place, PlacedWindow, Placement},
    Phase, Runner, TickSpan, FOREIGN_COLLECTOR, NEVER_SET_UP,
};
use crate::{
    assertion::{
        evaluator::{earliest_failure, evaluate_reduced},
        Assertion, AssertionResult, Measure, Window,
    },
    collector::{instance_of, ObservationCollector, Sample},
    report::{
        AssertionReportEntry, FailureReason, MeasureReportEntry, Report, SeriesSummary,
        TerminatedBy, Verdict,
    },
    run::termination::TerminationReason,
    statistics::within_run::{latest, max_gap, Reducer},
};
use helios_core::prelude::{AgentId, MonotonicTime};
use helios_runtime::{observe::key::RecordedKey, port::RefusedWrites};

impl Runner {
    /// Build the run's [`Report`] once the host stops ticking. Every entry is
    /// evaluated here, once, on the samples the run reached that are stamped
    /// in its window: an every-sample assertion on each of them, placing a
    /// failure at the earliest failing stamp (the latch `tick` folded only
    /// decides `fail_fast`); a reduced assertion and a measure on their one
    /// reduced value.
    ///
    /// Each window is placed on the run's clock and covered once the run's
    /// latest tick reached its end. A window bounded from `run.end`, and the
    /// whole run, are covered only by a run that reached its time budget.
    /// The runner reads that from its own clock, not from the stop reason
    /// the host passes, which only labels the report: a `fail_fast` failure
    /// on the tick the budget runs out still covered it. An assertion whose
    /// window wasn't covered, or can't be placed (a `run.end` bound after an
    /// early stop, or an anchor that hadn't occurred yet), and that hadn't
    /// failed in it, is `Unjudged`: neither its pass nor its silence says
    /// anything about a window the run never finished, and a measure is
    /// marked not covered. A failure is final however the run stopped. In a
    /// covered window, an assertion with no samples to judge fails, since
    /// that is how a dropped signal looks; in a run that ran out, one whose
    /// anchor never occurred fails too.
    ///
    /// The run is `Passed` only if every assertion passed, setup didn't fail,
    /// no agent's bus refused a write ([`HostFacts::refused_writes`]) and the
    /// recording dropped nothing: no requested value arrived under an
    /// unbound key ([`ObservationCollector::unbound_arrivals`]) and no
    /// measure's series turned a value away. Values the wiring or the
    /// recording dropped leave the entries describing another run than the
    /// one configured. Only a run that was set up reports the recording's
    /// drops; in one aborted at setup, a refused entry's key is unbound by
    /// design.
    ///
    /// Takes `&self`, not `&mut`: the latches are only read, nothing is
    /// recorded. Everything else the report needs comes from the host as
    /// [`HostFacts`].
    pub fn finalize(&self, host: HostFacts, collector: &ObservationCollector) -> Report {
        let HostFacts {
            sim_now,
            wall_duration_secs,
            run_name,
            terminated_by_reason,
            master_seed,
            refused_writes,
        } = host;

        // A host that finalizes without having set the run up, or with
        // another run's collector, fails it here as `tick` would have.
        let mut setup_failures = self.setup_failures.clone();
        let plan = match &self.phase {
            Phase::Ready(plan) => Some(plan),
            Phase::AwaitingPipelines | Phase::Aborted => None,
        };
        let judged = match &self.phase {
            Phase::Ready(plan) => {
                let judged = plan.with_series(collector);
                if judged.is_none() {
                    setup_failures.push(FOREIGN_COLLECTOR.to_string());
                }
                judged
            }
            Phase::AwaitingPipelines => {
                setup_failures.push(NEVER_SET_UP.to_string());
                None
            }
            Phase::Aborted => None,
        };
        let judged = judged.unwrap_or_default();
        // The entries `owner` owns, one per series it reads; none when the
        // run isn't ready to judge.
        let entries_of = |owner: EntryOwner| -> Vec<&EntryView<'_, '_, &AssertionState>> {
            judged.iter().filter(|entry| entry.owner == owner).collect()
        };
        let ran_out = self
            .ticks
            .is_some_and(|span| self.run.termination().budget_spent(span.first, span.latest));
        // Where `window` falls in the run, each anchor at its series' first
        // stamp in the run.
        let place_in = |window: &Window, span: TickSpan| {
            place(window, span, ran_out, |after| {
                let series = plan?.anchor(after, collector)?;
                first_stamp(series.samples(), span.latest)
            })
        };
        // An entry the run never read still gets its line.
        let each_or_unread = |owner| {
            let entries = entries_of(owner);
            if entries.is_empty() {
                vec![None]
            } else {
                entries.into_iter().map(Some).collect()
            }
        };

        let assertions: Vec<AssertionReportEntry> = self
            .run
            .assertions()
            .iter()
            .enumerate()
            .flat_map(|(index, assertion)| {
                each_or_unread(EntryOwner::Assertion(index))
                    .into_iter()
                    .map(|entry| {
                        let (series, verdict) = match entry.zip(self.ticks) {
                            Some((entry, span)) => {
                                let windowed = Windowed::new(
                                    entry,
                                    &assertion.window,
                                    place_in(&assertion.window, span),
                                    span,
                                );
                                (
                                    windowed.summary(&assertion.reduce),
                                    judge(assertion, entry.state, &windowed),
                                )
                            }
                            None => (unread(), Verdict::Unjudged { actual: None }),
                        };
                        AssertionReportEntry::new(
                            assertion.name.to_string(),
                            assertion.target.clone(),
                            entry.and_then(|entry| instance(entry.key)),
                            assertion.window.clone(),
                            assertion.reduce.clone(),
                            assertion.condition.clone(),
                            series,
                            verdict,
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .collect();

        let mut recording: Vec<String> = match &self.phase {
            Phase::Ready(_) => collector
                .unbound_arrivals()
                .iter()
                .map(ToString::to_string)
                .collect(),
            Phase::AwaitingPipelines | Phase::Aborted => Vec::new(),
        };
        // A measure's value never fails the run, but a value its series
        // turned away is a fault in the recording, like an unbound arrival.
        let measures: Vec<MeasureReportEntry> = self
            .run
            .measures()
            .iter()
            .enumerate()
            .flat_map(|(index, measure)| {
                each_or_unread(EntryOwner::Measure(index))
                    .into_iter()
                    .map(|judged| {
                        if let Some(entry) = judged {
                            if let Some(rejection) = entry.state.rejection() {
                                recording.push(format!(
                                    "measure '{}' on '{}': value rejected: {rejection}",
                                    measure.name,
                                    spelled(entry.key)
                                ));
                            }
                        }
                        let windowed = judged.zip(self.ticks).map(|(entry, span)| {
                            Windowed::new(
                                entry,
                                &measure.window,
                                place_in(&measure.window, span),
                                span,
                            )
                        });
                        report_measure(measure, judged, windowed.as_ref())
                    })
                    .collect::<Vec<_>>()
            })
            .collect();

        let refused_writes: Vec<String> = refused_writes
            .into_iter()
            .map(|(agent, refused)| format!("agent '{agent}': {refused}"))
            .collect();

        // Elapsed simulated time from the origin `tick` stamped on its first
        // call. `None` only if finalize is reached with no tick having run;
        // report zero rather than panic.
        let simulated_duration_secs = match self.ticks {
            Some(span) => sim_now.0 - span.first.0,
            None => 0.0f64,
        };

        Report::new(
            run_name,
            self.run.scenario().path().to_string(),
            master_seed,
            setup_failures,
            refused_writes,
            recording,
            simulated_duration_secs,
            wall_duration_secs,
            TerminatedBy::from(&terminated_by_reason),
            assertions,
            measures,
        )
    }
}

/// What a finished run's report needs that only the host knows: the runner
/// reads no clock, stores no name, does not own the seed and holds no
/// pipeline.
pub struct HostFacts {
    /// The tick that ended the run, not the clock as it stands at finalize.
    pub sim_now: MonotonicTime,
    pub wall_duration_secs: f64,
    pub run_name: String,
    /// Why the host stopped ticking. It only labels the report; whether the
    /// run covered its windows is read from the runner's own clock.
    pub terminated_by_reason: TerminationReason,
    /// `None` for an unseeded run, whose real seed is chosen inside the sim
    /// and unknown here.
    pub master_seed: Option<u64>,
    /// Every write each agent's bus refused, read from its pipeline.
    pub refused_writes: Vec<(AgentId, RefusedWrites)>,
}

/// One entry's series cut to its window: where the window fell, and the
/// samples the run reached stamped in it, a held series' carried-in value
/// first.
struct Windowed {
    placement: Placement,
    /// In the window when it was placed; every sample the run reached when
    /// it wasn't.
    samples: Vec<Sample>,
    /// The run's ticks: what times are reported from, and the edges of an
    /// unplaced window's summary.
    span: TickSpan,
    /// Whether the run file bounded the window, so the report shows it.
    bounded: bool,
}

impl Windowed {
    fn new(
        entry: &EntryView<'_, '_, &AssertionState>,
        window: &Window,
        placement: Placement,
        span: TickSpan,
    ) -> Self {
        let reached = entry.state.reached(entry.series.samples());
        let samples = match placement {
            Placement::Placed { window, .. } => window.samples(reached, entry.held),
            Placement::Unplaced | Placement::AnchorNeverOccurred => reached.to_vec(),
        };
        Self {
            placement,
            samples,
            span,
            bounded: !window.is_whole_run(),
        }
    }

    /// The placed window, if it was placed.
    fn window(&self) -> Option<PlacedWindow> {
        match self.placement {
            Placement::Placed { window, .. } => Some(window),
            Placement::Unplaced | Placement::AnchorNeverOccurred => None,
        }
    }

    /// Whether the run's latest tick reached the window's end.
    fn covered(&self) -> bool {
        matches!(self.placement, Placement::Placed { covered: true, .. })
    }

    /// Seconds from the run's first tick to `t`.
    fn secs(&self, t: MonotonicTime) -> f64 {
        t.0 - self.span.first.0
    }

    /// What the samples held over the window: how many, the largest gap, and,
    /// for `final`, how long before the window's end the judged sample was
    /// stamped. An unplaced window's summary is over the run's span.
    fn summary(&self, reduce: &Option<Reducer>) -> SeriesSummary {
        let (start, end) = match self.window() {
            Some(window) => (window.start, window.end),
            None => (self.span.first, self.span.latest),
        };
        let final_age_secs = match reduce {
            Some(Reducer::Final) => {
                latest(&self.samples).map(|sample| (end.0 - sample.timestamp.0).max(0.0))
            }
            _ => None,
        };
        let window_secs = self
            .window()
            .filter(|_| self.bounded)
            .map(|window| [self.secs(window.start), self.secs(window.end)]);
        SeriesSummary {
            samples: self.samples.len(),
            max_gap_secs: Some(max_gap(&self.samples, start, end)),
            final_age_secs,
            window_secs,
        }
    }
}

/// One assertion's report verdict. A value its series turned away fails it
/// whatever its window. Otherwise the window decides: unplaced is
/// `Unjudged`, an anchor that never occurred fails, and a placed window is
/// judged on its samples, a failure final either way and a pass or silence
/// only when the run covered it.
fn judge(assertion: &Assertion, state: &AssertionState, windowed: &Windowed) -> Verdict {
    if state.rejection().is_some() {
        return Verdict::Failed {
            actual: None,
            reason: FailureReason::Rejected,
            failed_at_secs: None,
        };
    }
    let window = match windowed.placement {
        Placement::Placed { window, .. } => window,
        Placement::Unplaced => return Verdict::Unjudged { actual: None },
        Placement::AnchorNeverOccurred => {
            return Verdict::Failed {
                actual: None,
                reason: FailureReason::AnchorNeverOccurred,
                failed_at_secs: None,
            }
        }
    };
    let samples = &windowed.samples;

    let Some(reducer) = &assertion.reduce else {
        // The value that broke it, at its own stamp, not whatever arrived
        // after: a run that recovered would otherwise show a healthy number
        // beside FAILED.
        if let Some((failing, kind)) = earliest_failure(&assertion.condition, samples) {
            return Verdict::Failed {
                actual: Some(failing.value.clone()),
                reason: FailureReason::from(&kind),
                failed_at_secs: Some(windowed.secs(failing.timestamp)),
            };
        }
        let last = latest(samples).map(|sample| sample.value.clone());
        if !windowed.covered() {
            return Verdict::Unjudged { actual: last };
        }
        // Silent over its whole window: a drop.
        return match last {
            Some(actual) => Verdict::Passed { actual },
            None => no_samples(),
        };
    };

    // Any other verdict belongs to an end the run never reached. The value
    // reduced so far is still shown.
    if !windowed.covered() {
        return Verdict::Unjudged {
            actual: reducer
                .reduce(samples, window.start, window.end)
                .ok()
                .flatten(),
        };
    }
    match evaluate_reduced(
        &assertion.condition,
        reducer,
        None,
        samples,
        window.start,
        window.end,
    ) {
        AssertionResult::Pending => no_samples(),
        judged => Verdict::from(judged),
    }
}

/// One measure's report line: its value reduced over its window, or why it
/// has none. `None` for both when the run never read it, or never placed its
/// window.
fn report_measure(
    measure: &Measure,
    judged: Option<&EntryView<'_, '_, &AssertionState>>,
    windowed: Option<&Windowed>,
) -> MeasureReportEntry {
    let rejected = judged.and_then(|entry| entry.state.rejection()).is_some();
    let value = match (rejected, windowed.map(|windowed| windowed.placement)) {
        (true, _) => Some(Err(FailureReason::Rejected)),
        (false, None | Some(Placement::Unplaced)) => None,
        (false, Some(Placement::AnchorNeverOccurred)) => {
            Some(Err(FailureReason::AnchorNeverOccurred))
        }
        (false, Some(Placement::Placed { window, .. })) => {
            let samples = windowed.map_or(&[][..], |windowed| &windowed.samples);
            Some(
                match measure.reduce.reduce(samples, window.start, window.end) {
                    Ok(Some(value)) => Ok(value),
                    Ok(None) => Err(FailureReason::NoSamples),
                    Err(_) => Err(FailureReason::TypeMismatch),
                },
            )
        }
    };
    MeasureReportEntry::new(
        measure.name.to_string(),
        measure.target.clone(),
        judged.and_then(|entry| instance(entry.key)),
        measure.window.clone(),
        measure.reduce.clone(),
        windowed.is_some_and(Windowed::covered),
        value,
        windowed.map_or_else(unread, |windowed| {
            windowed.summary(&Some(measure.reduce.clone()))
        }),
    )
}

/// The instance `key` records, spelled for the report; `None` on a series
/// that isn't one instance of a keyed channel.
fn instance(key: &RecordedKey) -> Option<String> {
    instance_of(&key.subject).map(str::to_owned)
}

/// `key`'s path, with its instance where it has one.
fn spelled(key: &RecordedKey) -> String {
    match instance(key) {
        Some(instance) => format!("{}[instance={instance}]", key.path()),
        None => key.path(),
    }
}

/// The summary of an entry the run never read: not set up, set up against
/// another collector, or never ticked.
fn unread() -> SeriesSummary {
    SeriesSummary {
        samples: 0,
        max_gap_secs: None,
        final_age_secs: None,
        window_secs: None,
    }
}

/// The verdict of an assertion that saw no value over its whole window.
fn no_samples() -> Verdict {
    Verdict::Failed {
        actual: None,
        reason: FailureReason::NoSamples,
        failed_at_secs: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::{AssertionValue, EntryName},
        report::{AssertionStatus, ReportStatus},
        runner::{
            fixtures::{
                at, cont, each_wheel, leaf_key, measure, pipeline, ready, reduced_below,
                refused_write, report, report_speed, report_unbound, report_unreadable,
                report_wheel, run, run_measuring, speed_target, speed_term, target, term, windowed,
                CAR, TRUCK,
            },
            TickAction,
        },
    };

    const BUDGET_SECS: f64 = 5.0;

    // `{:?}` forces a decimal point so TOML reads it as a float.
    fn budget() -> String {
        format!("max_simulated_seconds = {BUDGET_SECS:?}")
    }

    // Tick from the run's start to its budget, so it covers its window. Each
    // tick reaches whatever has arrived by then.
    fn run_out(r: &mut Runner, collector: &ObservationCollector) {
        r.tick(at(0.0), collector);
        r.tick(at(BUDGET_SECS), collector);
    }

    // Build the report the way `finalize_system` does, for a run the host
    // says ran out its budget.
    fn finalize(r: &Runner, collector: &ObservationCollector) -> Report {
        finalize_stopped_by(r, collector, TerminationReason::MaxSimulatedSeconds)
    }

    fn finalize_stopped_by(
        r: &Runner,
        collector: &ObservationCollector,
        reason: TerminationReason,
    ) -> Report {
        r.finalize(host_facts(reason, Vec::new()), collector)
    }

    // What the host hands finalize: a run named "run", stopped by `reason`,
    // whose buses refused `refused_writes`.
    fn host_facts(
        reason: TerminationReason,
        refused_writes: Vec<(AgentId, RefusedWrites)>,
    ) -> HostFacts {
        HostFacts {
            sim_now: at(0.0),
            wall_duration_secs: 0.0,
            run_name: "run".to_string(),
            terminated_by_reason: reason,
            master_seed: None,
            refused_writes,
        }
    }

    /// The stop for the assertion named `first` failing on the car's leaf.
    fn failed_first() -> TerminationReason {
        TerminationReason::AssertionFailed {
            name: EntryName::parse("first").expect("valid name"),
            key: leaf_key(CAR),
        }
    }

    fn statuses(report: &Report) -> Vec<AssertionStatus> {
        report
            .assertions()
            .iter()
            .map(|entry| entry.status())
            .collect()
    }

    #[test]
    fn a_final_assertion_is_judged_on_the_latest_sample_reached() {
        // The 9.0 arrives after the run's last tick, past its end, so the
        // verdict rests on the 5.0 the run reached.
        let (mut r, mut collector) = ready(run(&budget(), &[&term("x1", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);
        report(&mut collector, CAR, 5.5, 9.0);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(
            report.assertions()[0].verdict().actual(),
            Some(&AssertionValue::Float(5.0))
        );
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn an_assertion_that_never_saw_a_sample_fails() {
        // "Never emitted" is how a silent drop looks, so it can't pass, for
        // either kind of assertion.
        let (mut r, collector) = ready(
            run(&budget(), &[&term("x2", CAR, 5.0), &cont("x3", CAR, 5.0)]),
            &[CAR],
        );
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Failed]
        );
        for entry in report.assertions() {
            assert_eq!(
                entry.verdict(),
                &Verdict::Failed {
                    actual: None,
                    reason: FailureReason::NoSamples,
                    failed_at_secs: None,
                }
            );
        }
        assert_eq!(report.status(), &ReportStatus::Failed);
    }

    #[test]
    fn an_every_sample_failure_reports_the_value_that_broke_it() {
        // It breaks at 1.0 s and recovers at 1.5 s; the report shows the
        // breaking value and its time, not the healthy value that came last.
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x4", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        report(&mut collector, CAR, 1.0, 9.0);
        report(&mut collector, CAR, 1.5, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);
        let entry = &report.assertions()[0];

        assert_eq!(
            entry.verdict(),
            &Verdict::Failed {
                actual: Some(AssertionValue::Float(9.0)),
                reason: FailureReason::ConditionFailed,
                failed_at_secs: Some(1.0),
            }
        );
    }

    #[test]
    fn an_early_stop_leaves_what_had_not_failed_unjudged() {
        // Assertion 1 failed and stopped the run. Assertion 2 held so far,
        // 3 never saw a sample, and 4 is reduced: none of them saw the end
        // of its window, so none is passed, or failed for silence.
        let assertions = [
            cont("first", CAR, 5.0),
            cont("x6", CAR, 9.0),
            cont("x7", TRUCK, 5.0),
            term("x8", CAR, 9.0),
        ];
        let blocks: Vec<&str> = assertions.iter().map(String::as_str).collect();
        let (mut r, mut collector) = ready(run(&budget(), &blocks), &[CAR, TRUCK]);
        report(&mut collector, CAR, 0.5, 9.0);
        r.tick(at(1.0), &collector);

        let report = finalize_stopped_by(&r, &collector, failed_first());

        assert_eq!(
            statuses(&report),
            [
                AssertionStatus::Failed,
                AssertionStatus::Unjudged,
                AssertionStatus::Unjudged,
                AssertionStatus::Unjudged,
            ]
        );
        assert_eq!(report.status(), &ReportStatus::Failed);
    }

    #[test]
    fn an_unjudged_reduced_assertion_shows_the_value_it_stopped_at() {
        let (mut r, mut collector) = ready(
            run(
                &budget(),
                &[&cont("first", CAR, 5.0), &term("x10", CAR, 5.0)],
            ),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 7.0);
        r.tick(at(1.0), &collector);

        let report = finalize_stopped_by(&r, &collector, failed_first());

        assert_eq!(
            report.assertions()[1].verdict().actual(),
            Some(&AssertionValue::Float(7.0))
        );
    }

    #[test]
    fn an_every_sample_assertion_reports_its_latch() {
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x11", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn a_failure_on_the_tick_the_budget_runs_out_leaves_the_rest_judged() {
        // The failure names the stop, but the run still reached its end, so
        // the truck's assertion passed rather than going unjudged.
        let termination = format!("{}, fail_fast = true", budget());
        let (mut r, mut collector) = ready(
            run(
                &termination,
                &[&cont("first", CAR, 5.0), &cont("x13", TRUCK, 5.0)],
            ),
            &[CAR, TRUCK],
        );
        r.tick(at(0.0), &collector);
        report(&mut collector, CAR, 4.5, 9.0);
        report(&mut collector, TRUCK, 4.5, 5.0);
        assert_eq!(
            r.tick(at(BUDGET_SECS), &collector),
            TickAction::Terminate {
                reason: failed_first(),
            }
        );

        let report = finalize_stopped_by(&r, &collector, failed_first());

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Passed]
        );
    }

    #[test]
    fn the_stop_reason_the_host_passes_does_not_decide_coverage() {
        // The host says the budget ran out, but the runner's own ticks only
        // reached 1 s of 5: a pass would be claimed for time never run.
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x14", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        r.tick(at(1.0), &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Unjudged]);
    }

    #[test]
    fn a_value_turned_away_before_the_end_fails_the_assertion() {
        let (mut r, mut collector) = ready(run(&budget(), &[&term("x15", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        report_unreadable(&mut collector, CAR, 1.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Failed]);
    }

    #[test]
    fn a_value_turned_away_after_the_last_tick_is_past_the_end() {
        // Like a late sample, a late rejection is outside the run.
        let (mut r, mut collector) = ready(run(&budget(), &[&term("x16", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);
        report_unreadable(&mut collector, CAR, 5.5);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
    }

    #[test]
    fn finalizing_with_another_runs_collector_fails_the_run() {
        let (mut r, mut own) = ready(run(&budget(), &[&term("x17", CAR, 5.0)]), &[CAR]);
        report(&mut own, CAR, 0.5, 5.0);
        r.tick(at(1.0), &own);
        let other = r.collector();

        let report = finalize(&r, &other);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert_eq!(report.setup_failures(), [FOREIGN_COLLECTOR]);
        assert_eq!(statuses(&report), [AssertionStatus::Unjudged]);
    }

    #[test]
    fn finalizing_a_run_never_set_up_fails_it() {
        // With no assertions there is nothing else to fail it: it would
        // otherwise report a pass for a run that never ran.
        let r = Runner::new(run(&budget(), &[])).expect("valid run");
        let collector = r.collector();

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert_eq!(report.setup_failures(), [NEVER_SET_UP]);
    }

    #[test]
    fn a_setup_failure_fails_a_run_with_no_assertions() {
        // The regression this guards: a scenario whose agent has no pipeline
        // usually has no assertion naming it either. Judged on assertions
        // alone, an empty set is a vacuous pass and the run reports success
        // for a scene that could never have been tested.
        let mut r = Runner::new(run(&budget(), &[])).expect("valid run");
        let collector = r.collector();
        r.fail_setup("agent 'car' has no autonomy pipeline".to_string());

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert!(report.assertions().is_empty());
    }

    #[test]
    fn a_setup_failure_is_reported_with_its_reason() {
        // status alone tells CI a run failed; without the reason there is
        // nothing in the report to say why, since no assertion failed.
        let mut r = Runner::new(run(&budget(), &[])).expect("valid run");
        let collector = r.collector();
        r.fail_setup("first".to_string());
        r.fail_setup("second".to_string());

        let report = finalize(&r, &collector);

        assert_eq!(report.setup_failures(), ["first", "second"]);
    }

    #[test]
    fn a_clean_run_records_no_setup_failures() {
        let (r, collector) = ready(run(&budget(), &[]), &[]);

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Passed);
        assert!(report.setup_failures().is_empty());
    }

    #[test]
    fn a_refused_write_fails_a_run_whose_assertions_passed() {
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x18", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);

        let report = r.finalize(
            host_facts(
                TerminationReason::MaxSimulatedSeconds,
                vec![refused_write(CAR)],
            ),
            &collector,
        );

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Failed);
        let [line] = report.refused_writes() else {
            panic!(
                "one refused channel, one line: {:?}",
                report.refused_writes()
            );
        };
        assert!(line.starts_with("agent 'car': "), "{line}");
        assert!(line.contains("\"undeclared\""), "{line}");
    }

    #[test]
    fn a_value_under_no_bound_series_fails_a_run_whose_assertions_passed() {
        // The speed channel's path is requested and bound as the pipeline
        // declares it, carrying a float; a value under it carrying a bool
        // lands in no series.
        let (mut r, mut collector) = ready(run(&budget(), &[&speed_term("x19", CAR, 5.0)]), &[CAR]);
        report_speed(&mut collector, CAR, 0.5, 5.0);
        report_unbound(&mut collector, CAR, 0.5);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Failed);
        let [line] = report.recording() else {
            panic!("one unbound key, one line: {:?}", report.recording());
        };
        assert!(line.starts_with("agent.car.channel.speed ("), "{line}");
        assert!(line.contains("bool"), "{line}");
        assert!(
            line.ends_with(": 1 value(s) arrived under no bound series"),
            "{line}"
        );
    }

    #[test]
    fn a_run_aborted_at_setup_reports_no_unbound_values() {
        // The truck's assertion is refused, so its key is never bound; values
        // under its path say nothing beyond the setup failure.
        let mut r = Runner::new(run(&budget(), &[&term("x20", TRUCK, 5.0)])).expect("valid run");
        let mut collector = r.collector();
        let pipeline = pipeline();
        assert!(r
            .on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &mut collector)
            .is_err());
        report(&mut collector, TRUCK, 0.5, 5.0);

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert!(report.recording().is_empty());
    }

    #[test]
    fn a_reduced_assertion_is_judged_on_its_reduced_value() {
        // The mean of 1, 2 and 6 is 3: below 4 though one sample isn't.
        let (mut r, mut collector) = ready(
            run(
                &budget(),
                &[&reduced_below("mean_low", CAR, "\"mean\"", 4.0)],
            ),
            &[CAR],
        );
        for (at, value) in [(0.5, 1.0), (1.0, 2.0), (1.5, 6.0)] {
            report(&mut collector, CAR, at, value);
        }
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            report.assertions()[0].verdict(),
            &Verdict::Passed {
                actual: AssertionValue::Float(3.0)
            }
        );
    }

    #[test]
    fn max_with_a_nan_sample_fails_as_judging_each_sample_would() {
        let (mut r, mut collector) = ready(
            run(
                &budget(),
                &[
                    &reduced_below("max_low", CAR, "\"max\"", 25.0),
                    &format!(
                        "name = \"each_low\"\ntarget = \"agent.{CAR}.node.estimator.x\"\n\
                         condition = {{ less_than = 25.0 }}"
                    ),
                ],
            ),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 1.0);
        report(&mut collector, CAR, 1.0, f64::NAN);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Failed]
        );
    }

    #[test]
    fn count_equals_zero_passes_on_a_silent_series() {
        // "Expect no samples": `count` is defined on an empty window.
        let none = format!(
            "name = \"never\"\ntarget = \"agent.{CAR}.node.estimator.x\"\n\
             reduce = \"count\"\ncondition = {{ equals = 0 }}"
        );
        let (mut r, collector) = ready(run(&budget(), &[&none]), &[CAR]);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            report.assertions()[0].verdict(),
            &Verdict::Passed {
                actual: AssertionValue::Int(0)
            }
        );
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn each_entry_reports_its_sample_count_largest_gap_and_final_age() {
        // Samples at 0.5 and 1.0 in a run from 0 to 5: the gap after the
        // last is 4 s, and the final sample is 4 s old at the end.
        let (mut r, mut collector) = ready(
            run(
                &budget(),
                &[&term("settled", CAR, 5.0), &cont("held", CAR, 5.0)],
            ),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 5.0);
        report(&mut collector, CAR, 1.0, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            report.assertions()[0].series(),
            &SeriesSummary {
                samples: 2,
                max_gap_secs: Some(4.0),
                final_age_secs: Some(4.0),
                window_secs: None,
            }
        );
        assert_eq!(
            report.assertions()[1].series(),
            &SeriesSummary {
                samples: 2,
                max_gap_secs: Some(4.0),
                final_age_secs: None,
                window_secs: None,
            }
        );
    }

    #[test]
    fn a_measure_reports_its_value_and_never_fails_the_run() {
        let (mut r, mut collector) = ready(
            run_measuring(
                &budget(),
                &[],
                &[
                    &measure("peak", CAR, "\"max\""),
                    &measure("mean_of_nothing", TRUCK, "\"mean\""),
                ],
            ),
            &[CAR, TRUCK],
        );
        report(&mut collector, CAR, 0.5, 2.0);
        report(&mut collector, CAR, 1.0, 7.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        let [peak, nothing] = report.measures() else {
            panic!("two measures: {:?}", report.measures());
        };
        assert_eq!(peak.value(), Some(&AssertionValue::Float(7.0)));
        assert!(peak.covered());
        assert_eq!(nothing.value(), None);
        assert_eq!(nothing.reason(), Some(FailureReason::NoSamples));
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn a_measure_the_run_did_not_cover_reports_the_value_it_stopped_at() {
        let (mut r, mut collector) = ready(
            run_measuring(&budget(), &[], &[&measure("peak", CAR, "\"max\"")]),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 2.0);
        r.tick(at(1.0), &collector);

        let report = finalize(&r, &collector);

        let [peak] = report.measures() else {
            panic!("one measure: {:?}", report.measures());
        };
        assert_eq!(peak.value(), Some(&AssertionValue::Float(2.0)));
        assert!(!peak.covered());
    }

    #[test]
    fn a_value_a_measures_series_turned_away_fails_the_run_under_recording() {
        // The assertion passes; the measure's value is no fault of the run's,
        // but the recording dropping one is.
        let (mut r, mut collector) = ready(
            run_measuring(
                &budget(),
                &[&cont("held", CAR, 5.0)],
                &[&measure("peak", CAR, "\"max\"")],
            ),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 5.0);
        report_unreadable(&mut collector, CAR, 1.0);
        r.tick(at(0.0), &collector);
        r.tick(at(BUDGET_SECS), &collector);

        let report = finalize(&r, &collector);

        assert_eq!(report.measures()[0].reason(), Some(FailureReason::Rejected));
        assert_eq!(report.status(), &ReportStatus::Failed);
        let [line] = report.recording() else {
            panic!("one recording line: {:?}", report.recording());
        };
        assert!(
            line.starts_with("measure 'peak' on 'agent.car.node.estimator.x': value rejected"),
            "{line}"
        );
    }

    /// Every sample, or the reduced value, is 5.
    const IS_FIVE: &str = "condition = { within_range = [5.0, 5.0] }";

    /// A window starting `plus` seconds after the car's speed is first
    /// written.
    fn after_speed(plus: f64) -> String {
        format!(
            "{{ start = {{ after = \"{}\", plus = {plus:?} }} }}",
            speed_target(CAR)
        )
    }

    #[test]
    fn an_every_sample_failure_is_placed_at_the_earliest_failing_stamp() {
        // The failure at 3 s arrives first; the one at 2 s is earlier.
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x30", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 3.0, 9.0);
        report(&mut collector, CAR, 2.0, 8.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            report.assertions()[0].verdict(),
            &Verdict::Failed {
                actual: Some(AssertionValue::Float(8.0)),
                reason: FailureReason::ConditionFailed,
                failed_at_secs: Some(2.0),
            }
        );
    }

    #[test]
    fn a_window_an_early_stop_passed_is_judged() {
        // `first` stops the run at 2 s; `early` ended at 1 s, so the run
        // covered it, and the failing sample at 1.5 s is outside it.
        let termination = format!("{}, fail_fast = true", budget());
        let early = windowed("early", &target(CAR), "{ end = 1.0 }", IS_FIVE);
        let (mut r, mut collector) = ready(
            run(&termination, &[&cont("first", CAR, 5.0), &early]),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 5.0);
        report(&mut collector, CAR, 1.5, 9.0);
        r.tick(at(0.0), &collector);
        r.tick(at(2.0), &collector);

        let report = finalize_stopped_by(&r, &collector, failed_first());

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Passed]
        );
        assert_eq!(
            report.assertions()[1].series().window_secs,
            Some([0.0, 1.0])
        );
    }

    #[test]
    fn a_window_from_run_end_is_judged_only_when_the_run_ran_out() {
        let last = windowed(
            "last",
            &target(CAR),
            "{ start = { at = \"run.end\", plus = -1.0 } }",
            IS_FIVE,
        );
        let termination = format!("{}, fail_fast = true", budget());
        let blocks = [cont("first", TRUCK, 5.0), last];
        let blocks: Vec<&str> = blocks.iter().map(String::as_str).collect();

        // Ran out: the last second holds only the 5.0.
        let (mut r, mut collector) = ready(run(&termination, &blocks), &[CAR, TRUCK]);
        report(&mut collector, TRUCK, 0.5, 5.0);
        report(&mut collector, CAR, 1.0, 9.0);
        report(&mut collector, CAR, 4.5, 5.0);
        run_out(&mut r, &collector);
        let ran_out = finalize(&r, &collector);
        assert_eq!(
            statuses(&ran_out),
            [AssertionStatus::Passed, AssertionStatus::Passed]
        );
        assert_eq!(
            ran_out.assertions()[1].series().window_secs,
            Some([4.0, 5.0])
        );

        // Stopped at 2 s: run.end isn't the run's end, so it isn't judged
        // over 1 s to 2 s.
        let (mut r, mut collector) = ready(run(&termination, &blocks), &[CAR, TRUCK]);
        report(&mut collector, TRUCK, 0.5, 9.0);
        r.tick(at(0.0), &collector);
        r.tick(at(2.0), &collector);
        let stopped = finalize_stopped_by(&r, &collector, failed_first());
        assert_eq!(
            statuses(&stopped),
            [AssertionStatus::Failed, AssertionStatus::Unjudged]
        );
    }

    #[test]
    fn an_anchored_window_judges_only_what_follows_the_event() {
        // Speed is first written at 2 s; the 9.0 at 1 s comes before the
        // window opens at 2.5 s.
        let settled = windowed("settled", &target(CAR), &after_speed(0.5), IS_FIVE);
        let (mut r, mut collector) = ready(run(&budget(), &[&settled]), &[CAR]);
        report_speed(&mut collector, CAR, 3.0, 1.0);
        report_speed(&mut collector, CAR, 2.0, 1.0);
        report(&mut collector, CAR, 1.0, 9.0);
        report(&mut collector, CAR, 3.0, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(
            report.assertions()[0].series().window_secs,
            Some([2.5, 5.0])
        );
        assert_eq!(report.assertions()[0].series().samples, 1);
    }

    #[test]
    fn an_anchor_that_never_occurred_fails_only_a_run_that_ran_out() {
        let settled = windowed("settled", &target(CAR), &after_speed(0.0), IS_FIVE);
        let (mut r, mut collector) = ready(run(&budget(), &[&settled]), &[CAR]);
        report(&mut collector, CAR, 1.0, 5.0);
        run_out(&mut r, &collector);
        assert_eq!(
            finalize(&r, &collector).assertions()[0].verdict(),
            &Verdict::Failed {
                actual: None,
                reason: FailureReason::AnchorNeverOccurred,
                failed_at_secs: None,
            }
        );

        let (mut r, collector) = ready(run(&budget(), &[&settled]), &[CAR]);
        r.tick(at(1.0), &collector);
        assert_eq!(
            statuses(&finalize_stopped_by(
                &r,
                &collector,
                TerminationReason::Aborted
            )),
            [AssertionStatus::Unjudged]
        );
    }

    #[test]
    fn a_window_ending_past_the_last_tick_is_unjudged_and_fails_the_run() {
        // The event at 4.5 s opens a window to 6.5 s; the run ran out at 5 s.
        let window = format!(
            "{{ end = {{ after = \"{}\", plus = 2.0 }} }}",
            speed_target(CAR)
        );
        let late = windowed("late", &target(CAR), &window, IS_FIVE);
        let (mut r, mut collector) = ready(run(&budget(), &[&late]), &[CAR]);
        report_speed(&mut collector, CAR, 4.5, 1.0);
        report(&mut collector, CAR, 1.0, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Unjudged]);
        assert_eq!(report.status(), &ReportStatus::Failed);
    }

    #[test]
    fn a_channel_carries_its_value_into_a_window_and_an_emission_does_not() {
        let from_two = "{ start = 2.0 }";
        let speed_count = windowed(
            "speed_count",
            &speed_target(CAR),
            from_two,
            "reduce = \"count\"",
        );
        let leaf_count = windowed("leaf_count", &target(CAR), from_two, "reduce = \"count\"");
        let (mut r, mut collector) = ready(
            run_measuring(&budget(), &[], &[&speed_count, &leaf_count]),
            &[CAR],
        );
        report_speed(&mut collector, CAR, 0.5, 3.0);
        report(&mut collector, CAR, 0.5, 3.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        let counts: Vec<Option<&AssertionValue>> = report
            .measures()
            .iter()
            .map(|entry| entry.value())
            .collect();
        assert_eq!(
            counts,
            [Some(&AssertionValue::Int(1)), Some(&AssertionValue::Int(0))]
        );
    }

    #[test]
    fn a_measure_on_an_anchor_that_never_occurred_says_so() {
        let peak = windowed("peak", &target(CAR), &after_speed(0.0), "reduce = \"max\"");
        let (mut r, mut collector) = ready(run_measuring(&budget(), &[], &[&peak]), &[CAR]);
        report(&mut collector, CAR, 1.0, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        let entry = &report.measures()[0];
        assert_eq!(entry.value(), None);
        assert_eq!(entry.reason(), Some(FailureReason::AnchorNeverOccurred));
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn final_age_is_measured_to_the_windows_end() {
        let to_three = windowed(
            "to_three",
            &target(CAR),
            "{ end = 3.0 }",
            &format!("reduce = \"final\"\n{IS_FIVE}"),
        );
        let (mut r, mut collector) = ready(run(&budget(), &[&to_three]), &[CAR]);
        report(&mut collector, CAR, 1.0, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(report.assertions()[0].series().final_age_secs, Some(2.0));
    }

    /// The instance each entry of `entries` judged, with its status.
    fn by_instance(report: &Report) -> Vec<(Option<&str>, AssertionStatus)> {
        report
            .assertions()
            .iter()
            .map(|entry| (entry.instance(), entry.status()))
            .collect()
    }

    #[test]
    fn each_instance_is_judged_and_reported_apart() {
        let each = format!(
            "name = \"slow\"\ntarget = {}\ncondition = {{ less_than = 5.0 }}",
            each_wheel(CAR)
        );
        let count = format!(
            "name = \"updates\"\ntarget = {}\nreduce = \"count\"",
            each_wheel(CAR)
        );
        let (mut r, mut collector) = ready(
            run_measuring(&budget(), &[&each, &cont("first", CAR, 5.0)], &[&count]),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 5.0);
        report_wheel(&mut collector, CAR, "front", 0.5, 1.0);
        report_wheel(&mut collector, CAR, "front", 1.0, 2.0);
        report_wheel(&mut collector, CAR, "rear", 0.5, 9.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        // In run-file order, each instance in the catalog's order.
        assert_eq!(
            by_instance(&report),
            [
                (Some("front"), AssertionStatus::Passed),
                (Some("rear"), AssertionStatus::Failed),
                (None, AssertionStatus::Passed),
            ]
        );
        assert!(report.assertions()[..2]
            .iter()
            .all(|entry| entry.name() == "slow"));
        let counts: Vec<(Option<&str>, Option<&AssertionValue>)> = report
            .measures()
            .iter()
            .map(|entry| (entry.instance(), entry.value()))
            .collect();
        assert_eq!(
            counts,
            [
                (Some("front"), Some(&AssertionValue::Int(2))),
                (Some("rear"), Some(&AssertionValue::Int(1))),
            ]
        );
        assert_eq!(report.status(), &ReportStatus::Failed);
    }

    #[test]
    fn an_assertion_on_each_instance_in_a_run_never_set_up_has_one_line() {
        let each = format!(
            "name = \"slow\"\ntarget = {}\ncondition = {{ less_than = 5.0 }}",
            each_wheel(CAR)
        );
        let r = Runner::new(run(&budget(), &[&each])).expect("valid run");
        let collector = r.collector();

        let report = finalize(&r, &collector);

        assert_eq!(by_instance(&report), [(None, AssertionStatus::Unjudged)]);
    }
}
