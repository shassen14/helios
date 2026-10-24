//! Where a run file's window falls in a run: its bounds placed on the run's
//! clock, whether the run covered it, and which samples it holds.

use super::TickSpan;
use crate::{
    assertion::{AssertionTarget, Bound, RunInstant, Window},
    collector::Sample,
};

use helios_core::prelude::MonotonicTime;

/// _Per-step._ A window placed in one run, or why it can't be.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Placement {
    /// Both bounds are placed. `covered` is whether the run's latest tick
    /// reached the window's end.
    Placed { window: PlacedWindow, covered: bool },
    /// The run stopped early, before an instant a bound needs: `run.end`,
    /// which is only the run's end when it ran out, or an anchor that might
    /// still have occurred.
    Unplaced,
    /// The run ran out and a series a bound is anchored `after` never
    /// arrived, so the window never existed.
    AnchorNeverOccurred,
}

/// Place `window` in a run whose ticks covered `span`. `ran_out` is whether
/// the run reached its own end (the budget); `anchor` gives the earliest
/// stamp of the series a selector names, or `None` if it has none.
pub fn place(
    window: &Window,
    span: TickSpan,
    ran_out: bool,
    anchor: impl Fn(&AssertionTarget) -> Option<MonotonicTime>,
) -> Placement {
    let mut never_occurred = false;
    let mut at = |bound: &Option<Bound>| -> Result<Option<MonotonicTime>, ()> {
        let Some(bound) = bound else {
            return Ok(None);
        };
        let (origin, plus) = match bound {
            Bound::Secs(secs) => (Some(span.first), *secs),
            Bound::At {
                at: RunInstant::Start,
                plus,
            } => (Some(span.first), *plus),
            Bound::At {
                at: RunInstant::End,
                plus,
            } => (ran_out.then_some(span.latest), *plus),
            Bound::After { after, plus } => {
                let occurred = anchor(after);
                never_occurred |= occurred.is_none() && ran_out;
                (occurred, *plus)
            }
        };
        origin
            .map(|origin| Some(MonotonicTime(origin.0 + plus)))
            .ok_or(())
    };
    let (start, end) = (at(&window.start), at(&window.end));
    let (Ok(from), Ok(until)) = (start, end) else {
        return if never_occurred {
            Placement::AnchorNeverOccurred
        } else {
            Placement::Unplaced
        };
    };

    // An absent end runs to `run.end`, which only a run that ran out
    // reached.
    let covered = match until {
        Some(until) => span.latest.0 >= until.0,
        None => ran_out,
    };
    let start = from.unwrap_or(span.first);
    let end = until.map_or(span.latest, |until| {
        MonotonicTime(until.0.min(span.latest.0))
    });
    Placement::Placed {
        window: PlacedWindow {
            start,
            end,
            from,
            until,
        },
        covered,
    }
}

/// _Per-step._ A window's bounds on the run's clock. The edges are what a
/// reducer measures gaps and held time from; the cuts are which samples
/// belong. They differ only where a bound is absent: it cuts nothing, while
/// its edge is the run's first or latest tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlacedWindow {
    /// The start edge: the start bound, or the run's first tick.
    pub start: MonotonicTime,
    /// The end edge: the end bound, never past the run's latest tick.
    pub end: MonotonicTime,
    /// The start cut. `None` takes every sample before.
    from: Option<MonotonicTime>,
    /// The end cut. `None` takes every sample after.
    until: Option<MonotonicTime>,
}

impl PlacedWindow {
    /// Whether a sample stamped `t` belongs to the window. Both ends are
    /// included.
    pub fn contains(&self, t: MonotonicTime) -> bool {
        self.from.is_none_or(|from| t.0 >= from.0) && self.until.is_none_or(|until| t.0 <= until.0)
    }

    /// The start cut, if the window has one.
    pub fn start_cut(&self) -> Option<MonotonicTime> {
        self.from
    }

    /// The samples of `samples` stamped in the window, in arrival order.
    /// A `held` series (a channel's) starts with the value in force at the
    /// start, [`carried_in`] at it.
    pub fn samples(&self, samples: &[Sample], held: bool) -> Vec<Sample> {
        let carried = held.then(|| self.carried_in(samples)).flatten();
        carried
            .into_iter()
            .chain(
                samples
                    .iter()
                    .filter(|sample| self.contains(sample.timestamp))
                    .cloned(),
            )
            .collect()
    }

    /// The value a held series carries into the window: its newest sample
    /// stamped before the start, restamped at the start. None when the
    /// window has no start, or a sample is stamped exactly there, since that
    /// one is the value in force.
    pub fn carried_in(&self, samples: &[Sample]) -> Option<Sample> {
        carried_in(samples, self.from?)
    }
}

/// The newest sample of `samples` stamped before `from`, restamped at
/// `from`, unless one is stamped exactly at `from`. Of several with the
/// newest stamp, the last to arrive.
fn carried_in(samples: &[Sample], from: MonotonicTime) -> Option<Sample> {
    if samples.iter().any(|sample| sample.timestamp.0 == from.0) {
        return None;
    }
    let before = samples
        .iter()
        .filter(|sample| sample.timestamp.0 < from.0)
        .reduce(|best, sample| {
            if sample.timestamp.0 >= best.timestamp.0 {
                sample
            } else {
                best
            }
        })?;
    Some(Sample {
        timestamp: from,
        value: before.value.clone(),
    })
}

/// The earliest stamp of `samples` at or before `latest`: when the event a
/// window is anchored `after` first happened in the run. Later stamps are
/// past the run's end.
pub fn first_stamp(samples: &[Sample], latest: MonotonicTime) -> Option<MonotonicTime> {
    samples
        .iter()
        .map(|sample| sample.timestamp)
        .filter(|t| t.0 <= latest.0)
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::AssertionValue;

    const EVENT: &str = "agent.car.node.faults.gps_outage";

    fn at(secs: f64) -> MonotonicTime {
        MonotonicTime(secs)
    }

    fn span(first: f64, latest: f64) -> TickSpan {
        TickSpan {
            first: at(first),
            latest: at(latest),
        }
    }

    fn sample(stamp: f64, value: f64) -> Sample {
        Sample {
            timestamp: at(stamp),
            value: AssertionValue::Float(value),
        }
    }

    fn window(text: &str) -> Window {
        #[derive(serde::Deserialize)]
        struct Wrap {
            window: Window,
        }
        toml::from_str::<Wrap>(&format!("window = {text}"))
            .expect("a window parses")
            .window
    }

    fn no_anchor(_: &AssertionTarget) -> Option<MonotonicTime> {
        None
    }

    fn placed(placement: Placement) -> (PlacedWindow, bool) {
        match placement {
            Placement::Placed { window, covered } => (window, covered),
            other => panic!("expected a placed window, got {other:?}"),
        }
    }

    #[test]
    fn the_whole_run_is_covered_only_when_the_run_ran_out() {
        let (whole, covered) = placed(place(&window("{}"), span(2.0, 12.0), true, no_anchor));
        assert!(covered);
        assert_eq!((whole.start, whole.end), (at(2.0), at(12.0)));
        assert!(whole.contains(at(0.0)) && whole.contains(at(99.0)));

        let (_, covered) = placed(place(&window("{}"), span(2.0, 12.0), false, no_anchor));
        assert!(!covered);
    }

    #[test]
    fn numbers_are_seconds_after_the_first_tick() {
        let (placed, covered) = placed(place(
            &window("{ start = 4.0, end = 8.0 }"),
            span(2.0, 7.0),
            false,
            no_anchor,
        ));
        // 6 s to 10 s on the run's clock; the run stopped at 7 s.
        assert!(!covered);
        assert_eq!(placed.start, at(6.0));
        assert_eq!(placed.end, at(7.0));
        assert!(!placed.contains(at(5.9)) && placed.contains(at(6.0)));
    }

    #[test]
    fn a_numeric_window_an_early_stop_passed_is_covered() {
        let (_, covered) = placed(place(
            &window("{ end = 10.0 }"),
            span(0.0, 12.0),
            false,
            no_anchor,
        ));
        assert!(covered);
    }

    #[test]
    fn run_end_is_placed_only_when_the_run_ran_out() {
        let last_five = window("{ start = { at = \"run.end\", plus = -5.0 } }");
        let (placed, covered) = placed(place(&last_five, span(0.0, 60.0), true, no_anchor));
        assert!(covered);
        assert_eq!((placed.start, placed.end), (at(55.0), at(60.0)));

        assert_eq!(
            place(&last_five, span(0.0, 12.0), false, no_anchor),
            Placement::Unplaced
        );
    }

    #[test]
    fn an_anchor_places_its_bound_after_the_event() {
        let outage = window(&format!(
            "{{ start = {{ after = \"{EVENT}\", plus = 2.0 }}, \
             end = {{ after = \"{EVENT}\", plus = 7.0 }} }}"
        ));
        let (placed, covered) = placed(place(&outage, span(0.0, 60.0), true, |_| Some(at(20.0))));
        assert!(covered);
        assert_eq!((placed.start, placed.end), (at(22.0), at(27.0)));
    }

    #[test]
    fn an_anchor_that_never_occurred_fails_only_a_run_that_ran_out() {
        let outage = window(&format!("{{ start = {{ after = \"{EVENT}\" }} }}"));
        assert_eq!(
            place(&outage, span(0.0, 60.0), true, no_anchor),
            Placement::AnchorNeverOccurred
        );
        assert_eq!(
            place(&outage, span(0.0, 12.0), false, no_anchor),
            Placement::Unplaced
        );
    }

    #[test]
    fn a_window_ending_past_the_last_tick_is_not_covered() {
        let outage = window(&format!(
            "{{ end = {{ after = \"{EVENT}\", plus = 7.0 }} }}"
        ));
        let (placed, covered) = placed(place(&outage, span(0.0, 60.0), true, |_| Some(at(58.0))));
        assert!(!covered);
        assert_eq!(placed.end, at(60.0));
    }

    #[test]
    fn a_held_series_carries_its_last_value_into_the_window() {
        let (from_ten, _) = placed(place(
            &window("{ start = 10.0 }"),
            span(0.0, 20.0),
            true,
            no_anchor,
        ));
        let samples = [sample(0.0, 1.0), sample(4.0, 2.0), sample(12.0, 3.0)];

        assert_eq!(
            from_ten.samples(&samples, true),
            [sample(10.0, 2.0), sample(12.0, 3.0)]
        );
        // An event carries nothing in.
        assert_eq!(from_ten.samples(&samples, false), [sample(12.0, 3.0)]);
    }

    #[test]
    fn a_value_stamped_at_the_start_is_not_carried_in_again() {
        let (from_ten, _) = placed(place(
            &window("{ start = 10.0 }"),
            span(0.0, 20.0),
            true,
            no_anchor,
        ));
        let samples = [sample(4.0, 2.0), sample(10.0, 3.0)];
        assert_eq!(from_ten.samples(&samples, true), [sample(10.0, 3.0)]);
    }

    #[test]
    fn a_window_without_a_start_carries_nothing_in() {
        let (to_ten, _) = placed(place(
            &window("{ end = 10.0 }"),
            span(0.0, 20.0),
            true,
            no_anchor,
        ));
        let samples = [sample(4.0, 2.0), sample(12.0, 3.0)];
        assert_eq!(to_ten.samples(&samples, true), [sample(4.0, 2.0)]);
    }

    #[test]
    fn an_anchor_is_its_series_earliest_stamp_in_the_run() {
        // Arrival order isn't stamp order; a stamp past the latest tick is
        // past the run.
        let samples = [sample(5.0, 1.0), sample(3.0, 1.0), sample(1.0, 1.0)];
        assert_eq!(first_stamp(&samples, at(4.0)), Some(at(1.0)));
        assert_eq!(first_stamp(&samples[..1], at(4.0)), None);
    }
}
