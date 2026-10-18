//! [`Windowed`]: one entry's series cut to its window, and what the
//! report says of the samples there.

use crate::{
    assertion::Window,
    collector::Sample,
    report::SeriesSummary,
    runner::{
        resolve::EntryView,
        state::AssertionState,
        window::{PlacedWindow, Placement},
        TickSpan,
    },
    statistics::within_run::{latest, max_gap, Reducer},
};

use helios_core::prelude::MonotonicTime;

/// One entry's series cut to its window: where the window fell, and the
/// samples the run reached stamped in it, a held series' carried-in value
/// first.
pub(super) struct Windowed {
    pub(super) placement: Placement,
    /// In the window when it was placed; every sample the run reached when
    /// it wasn't.
    pub(super) samples: Vec<Sample>,
    /// The run's ticks: what times are reported from, and the edges of an
    /// unplaced window's summary.
    span: TickSpan,
    /// Whether the run file bounded the window, so the report shows it.
    bounded: bool,
}

impl Windowed {
    pub(super) fn new(
        entry: &EntryView<'_, '_, &AssertionState>,
        window: &Window,
        placement: Placement,
        span: TickSpan,
    ) -> Self {
        // A derived series was computed from what the run reached, whole.
        let reached = if entry.is_derived() {
            entry.series.samples()
        } else {
            entry.state.reached(entry.series.samples())
        };
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
    pub(super) fn window(&self) -> Option<PlacedWindow> {
        match self.placement {
            Placement::Placed { window, .. } => Some(window),
            Placement::Unplaced | Placement::AnchorNeverOccurred => None,
        }
    }

    /// Whether the run's latest tick reached the window's end.
    pub(super) fn covered(&self) -> bool {
        matches!(self.placement, Placement::Placed { covered: true, .. })
    }

    /// Seconds from the run's first tick to `t`.
    pub(super) fn secs(&self, t: MonotonicTime) -> f64 {
        t.0 - self.span.first.0
    }

    /// What the samples held over the window: how many, the largest gap, and,
    /// for `final`, how long before the window's end the judged sample was
    /// stamped. An unplaced window's summary is over the run's span.
    pub(super) fn summary(&self, reduce: &Option<Reducer>) -> SeriesSummary {
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

#[cfg(test)]
mod tests {
    use crate::runner::finalize::fixtures::{
        after_speed, budget, failed_first, finalize, finalize_stopped_by, run_out, statuses,
        IS_FIVE,
    };

    use crate::{
        assertion::AssertionValue,
        report::{AssertionStatus, FailureReason, ReportStatus, SeriesSummary, Verdict},
        run::termination::TerminationReason,
        runner::fixtures::{
            at, cont, ready, report, report_speed, run, run_measuring, speed_target, target, term,
            windowed, CAR, TRUCK,
        },
    };

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
}
