//! Within-run reducers: one series' samples over a window, reduced to the one
//! value an assertion judges or a measure reports.
//!
//! Distinct from the cross-run [`StatisticTable`](super::StatisticTable),
//! which reduces a bag of finished runs' numbers: these take timed samples
//! and the window they fall in, since `final`, `time_above` and `max_gap`
//! need the stamps, and each keeps the kind its value has (`count` is an
//! int; `min`, `max` and `final` keep the series' kind). They also differ on
//! NaN: here a NaN sample reaches the reduced value, so `max` of a series
//! holding a NaN fails a bound just as judging each sample would, where the
//! cross-run `min` and `max` skip it.

use crate::{
    assertion::{AssertionValue, ValueKind},
    collector::Sample,
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::MonotonicTime;

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// _Shared (pre + post)._ How a series is reduced to one value. Written
/// `reduce = "mean"`, or `reduce = { time_above = 1.5 }` for a reducer with a
/// parameter. A closed set: a reducer is math, not a payload type, so the
/// set doesn't grow with the codebase, and a typo is refused at parse.
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Reducer {
    /// The sample with the latest stamp; a tie goes to the later arrival.
    Final,
    /// How many samples, as an int. `0` on an empty window.
    Count,
    /// The smallest sample, kept as an int when every sample is one.
    Min,
    /// The largest sample, kept as an int when every sample is one.
    Max,
    /// The mean per sample, not weighted by time.
    Mean,
    /// The population standard deviation (divided by N, as the cross-run
    /// table's), so one sample gives `0.0`.
    Std,
    /// The root mean square.
    Rms,
    /// Seconds the series spent above the threshold, each sample held until
    /// the next one or the window's end. Holding is right for a channel's
    /// state and wrong for an emission's events, so setup refuses it on an
    /// emission. `0.0` on an empty window.
    TimeAbove(f64),
    /// The longest span with no sample, the window's edges included. The
    /// window's length on an empty window.
    MaxGap,
}

impl Reducer {
    /// The reducer's name as a run file spells it.
    pub fn name(&self) -> &'static str {
        match self {
            Reducer::Final => "final",
            Reducer::Count => "count",
            Reducer::Min => "min",
            Reducer::Max => "max",
            Reducer::Mean => "mean",
            Reducer::Std => "std",
            Reducer::Rms => "rms",
            Reducer::TimeAbove(_) => "time_above",
            Reducer::MaxGap => "max_gap",
        }
    }

    /// The kind of the value this reducer gives over a series of kind
    /// `series`, or why it gives none: a number reducer on a bool or a
    /// string. Known at setup, so the condition is checked against the value
    /// it will judge.
    pub fn value_kind(&self, series: ValueKind) -> Result<ValueKind, ReduceError> {
        let numbers = || {
            if series.is_number() {
                Ok(())
            } else {
                Err(ReduceError::NotANumber {
                    reducer: self.name(),
                    value_kind: series,
                })
            }
        };
        match self {
            Reducer::Final => Ok(series),
            Reducer::Count => Ok(ValueKind::Int),
            Reducer::MaxGap => Ok(ValueKind::Float),
            Reducer::Min | Reducer::Max => numbers().map(|()| series),
            Reducer::Mean | Reducer::Std | Reducer::Rms | Reducer::TimeAbove(_) => {
                numbers().map(|()| ValueKind::Float)
            }
        }
    }

    /// The unit of the value this reducer gives over a series measured in
    /// `series`, `None` where it is undeclared. A count is a number of
    /// samples and a span is in seconds, whatever the series holds; every
    /// other reducer keeps the series' unit.
    pub fn si_unit(&self, series: Option<SiUnit>) -> Option<SiUnit> {
        match self {
            Reducer::Count => Some(SiUnit::Dimensionless),
            Reducer::TimeAbove(_) | Reducer::MaxGap => Some(SiUnit::Seconds),
            Reducer::Final
            | Reducer::Min
            | Reducer::Max
            | Reducer::Mean
            | Reducer::Std
            | Reducer::Rms => series,
        }
    }

    /// `samples`, in arrival order, reduced over the window from `start` to
    /// `end`. `Ok(None)` when the reducer has no value on an empty window;
    /// `Err` when it needs numbers and a sample isn't one.
    pub fn reduce(
        &self,
        samples: &[Sample],
        start: MonotonicTime,
        end: MonotonicTime,
    ) -> Result<Option<AssertionValue>, ReduceError> {
        match self {
            Reducer::Final => Ok(latest(samples).map(|sample| sample.value.clone())),
            Reducer::Count => Ok(Some(AssertionValue::Int(count(samples)))),
            Reducer::Min => self.extreme(samples, Extreme::Min),
            Reducer::Max => self.extreme(samples, Extreme::Max),
            Reducer::Mean => Ok(mean(&self.numbers(samples)?).map(AssertionValue::Float)),
            Reducer::Std => Ok(std(&self.numbers(samples)?).map(AssertionValue::Float)),
            Reducer::Rms => Ok(rms(&self.numbers(samples)?).map(AssertionValue::Float)),
            Reducer::TimeAbove(threshold) => {
                self.numbers(samples)?;
                Ok(Some(AssertionValue::Float(time_above(
                    samples, *threshold, start, end,
                ))))
            }
            Reducer::MaxGap => Ok(Some(AssertionValue::Float(max_gap(samples, start, end)))),
        }
    }

    /// Every sample as a number, in arrival order, or the first that isn't
    /// one.
    fn numbers(&self, samples: &[Sample]) -> Result<Vec<f64>, ReduceError> {
        samples
            .iter()
            .map(|sample| {
                sample.value.as_number().ok_or(ReduceError::NotANumber {
                    reducer: self.name(),
                    value_kind: sample.value.kind(),
                })
            })
            .collect()
    }

    /// The smallest or largest sample: an int when every sample is one, a
    /// float otherwise, NaN when any sample is NaN.
    fn extreme(
        &self,
        samples: &[Sample],
        extreme: Extreme,
    ) -> Result<Option<AssertionValue>, ReduceError> {
        let numbers = self.numbers(samples)?;
        let ints: Option<Vec<i64>> = samples
            .iter()
            .map(|sample| match sample.value {
                AssertionValue::Int(x) => Some(x),
                _ => None,
            })
            .collect();
        if let Some(ints) = ints {
            let found = match extreme {
                Extreme::Min => ints.into_iter().min(),
                Extreme::Max => ints.into_iter().max(),
            };
            return Ok(found.map(AssertionValue::Int));
        }
        if numbers.iter().any(|x| x.is_nan()) {
            return Ok(Some(AssertionValue::Float(f64::NAN)));
        }
        let found = numbers.into_iter().reduce(|a, b| match extreme {
            Extreme::Min => a.min(b),
            Extreme::Max => a.max(b),
        });
        Ok(found.map(AssertionValue::Float))
    }
}

/// Which end of a series [`Reducer::extreme`] takes.
#[derive(Clone, Copy)]
enum Extreme {
    Min,
    Max,
}

/// _Per-step._ Why a series couldn't be reduced.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ReduceError {
    #[error("{reducer} needs numbers, not values of kind {value_kind}")]
    NotANumber {
        reducer: &'static str,
        value_kind: ValueKind,
    },
}

/// The sample with the latest stamp; of several with that stamp, the last
/// to arrive. Arrival order isn't stamp order: a measurement update is
/// stamped with its reading's time and can arrive after a later-stamped
/// value.
pub fn latest(samples: &[Sample]) -> Option<&Sample> {
    samples.iter().reduce(|best, sample| {
        if sample.timestamp.0 >= best.timestamp.0 {
            sample
        } else {
            best
        }
    })
}

/// The longest span from `start` to `end` with no sample, the window's edges
/// included: the time before the first sample, between each two and after
/// the last. The window's length when there are none. A sample stamped
/// outside the window is held to its edge, so no gap is negative.
pub fn max_gap(samples: &[Sample], start: MonotonicTime, end: MonotonicTime) -> f64 {
    let mut stamps: Vec<f64> = samples
        .iter()
        .map(|sample| sample.timestamp.0.clamp(start.0, end.0.max(start.0)))
        .collect();
    stamps.sort_by(f64::total_cmp);

    let mut previous = start.0;
    let mut longest: f64 = 0.0;
    for stamp in stamps.into_iter().chain([end.0]) {
        longest = longest.max(stamp - previous);
        previous = stamp;
    }
    longest
}

fn count(samples: &[Sample]) -> i64 {
    i64::try_from(samples.len()).unwrap_or(i64::MAX)
}

fn mean(numbers: &[f64]) -> Option<f64> {
    if numbers.is_empty() {
        return None;
    }
    Some(numbers.iter().sum::<f64>() / numbers.len() as f64)
}

fn std(numbers: &[f64]) -> Option<f64> {
    let m = mean(numbers)?;
    let variance = numbers.iter().map(|x| (x - m).powi(2)).sum::<f64>() / numbers.len() as f64;
    Some(variance.sqrt())
}

fn rms(numbers: &[f64]) -> Option<f64> {
    let squares: Vec<f64> = numbers.iter().map(|x| x * x).collect();
    mean(&squares).map(f64::sqrt)
}

/// Seconds in the window `samples` spend above `threshold`, each held from
/// its stamp until the next sample's stamp or `end`. Nothing is held before
/// the first sample. NaN when any held sample is NaN: a NaN compares false
/// with every threshold, so its span would otherwise count as "not above"
/// and pass unseen. Every sample must be a number; the caller checks.
fn time_above(samples: &[Sample], threshold: f64, start: MonotonicTime, end: MonotonicTime) -> f64 {
    let end = end.0.max(start.0);
    let mut held: Vec<(f64, f64)> = samples
        .iter()
        .filter_map(|sample| {
            let value = sample.value.as_number()?;
            Some((sample.timestamp.0.clamp(start.0, end), value))
        })
        .collect();
    // Stable, so of two samples with one stamp the later arrival holds.
    held.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut above = 0.0;
    for (i, (from, value)) in held.iter().enumerate() {
        let until = held.get(i + 1).map_or(end, |next| next.0);
        let span = until - from;
        if span <= 0.0 {
            continue;
        }
        if value.is_nan() {
            return f64::NAN;
        }
        if *value > threshold {
            above += span;
        }
    }
    above
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(secs: f64) -> MonotonicTime {
        MonotonicTime(secs)
    }

    fn float(stamp: f64, value: f64) -> Sample {
        Sample {
            timestamp: at(stamp),
            value: AssertionValue::Float(value),
        }
    }

    fn int(stamp: f64, value: i64) -> Sample {
        Sample {
            timestamp: at(stamp),
            value: AssertionValue::Int(value),
        }
    }

    fn reduce(reducer: Reducer, samples: &[Sample]) -> Option<AssertionValue> {
        reducer
            .reduce(samples, at(0.0), at(10.0))
            .expect("numbers reduce")
    }

    fn reduced_float(reducer: Reducer, samples: &[Sample]) -> f64 {
        match reduce(reducer, samples) {
            Some(AssertionValue::Float(x)) => x,
            other => panic!("expected a float, got {other:?}"),
        }
    }

    #[test]
    fn a_reducer_parses_as_a_string_or_a_one_key_table() {
        #[derive(Debug, Deserialize)]
        struct Wrap {
            reduce: Reducer,
        }
        let parse = |text: &str| toml::from_str::<Wrap>(text).map(|wrap| wrap.reduce);
        assert_eq!(parse("reduce = \"count\"").unwrap(), Reducer::Count);
        assert_eq!(parse("reduce = \"max_gap\"").unwrap(), Reducer::MaxGap);
        assert_eq!(
            parse("reduce = { time_above = 1.5 }").unwrap(),
            Reducer::TimeAbove(1.5)
        );
        assert!(parse("reduce = \"maxx\"").is_err());
        assert!(parse("reduce = \"time_above\"").is_err());
    }

    #[test]
    fn on_an_empty_window_only_count_time_above_and_max_gap_have_a_value() {
        assert_eq!(reduce(Reducer::Count, &[]), Some(AssertionValue::Int(0)));
        assert_eq!(
            reduce(Reducer::TimeAbove(1.0), &[]),
            Some(AssertionValue::Float(0.0))
        );
        assert_eq!(
            reduce(Reducer::MaxGap, &[]),
            Some(AssertionValue::Float(10.0))
        );
        for reducer in [
            Reducer::Final,
            Reducer::Min,
            Reducer::Max,
            Reducer::Mean,
            Reducer::Std,
            Reducer::Rms,
        ] {
            assert_eq!(reduce(reducer.clone(), &[]), None, "{reducer:?}");
        }
    }

    #[test]
    fn final_is_the_latest_stamp_not_the_last_arrival() {
        assert_eq!(
            reduce(Reducer::Final, &[float(2.0, 9.0), float(1.0, 1.0)]),
            Some(AssertionValue::Float(9.0))
        );
    }

    #[test]
    fn final_breaks_a_tie_by_the_later_arrival() {
        assert_eq!(
            reduce(Reducer::Final, &[float(2.0, 9.0), float(2.0, 1.0)]),
            Some(AssertionValue::Float(1.0))
        );
    }

    #[test]
    fn count_is_an_int() {
        assert_eq!(
            reduce(Reducer::Count, &[float(1.0, 0.5), float(2.0, 0.5)]),
            Some(AssertionValue::Int(2))
        );
    }

    #[test]
    fn min_and_max_keep_an_int_series_an_int() {
        let counter = [int(1.0, 3), int(2.0, 1), int(3.0, 2)];
        assert_eq!(reduce(Reducer::Max, &counter), Some(AssertionValue::Int(3)));
        assert_eq!(reduce(Reducer::Min, &counter), Some(AssertionValue::Int(1)));
    }

    #[test]
    fn mean_std_and_rms_of_ints_are_floats() {
        let counter = [int(1.0, 1), int(2.0, 3)];
        assert_eq!(reduced_float(Reducer::Mean, &counter), 2.0);
        assert_eq!(reduced_float(Reducer::Std, &counter), 1.0);
        assert_eq!(reduced_float(Reducer::Rms, &counter), 5.0_f64.sqrt());
    }

    #[test]
    fn std_of_one_sample_is_zero() {
        assert_eq!(reduced_float(Reducer::Std, &[float(1.0, 4.0)]), 0.0);
    }

    #[test]
    fn a_nan_sample_reaches_every_number_reducer() {
        let series = [float(1.0, 1.0), float(2.0, f64::NAN), float(3.0, 2.0)];
        for reducer in [
            Reducer::Min,
            Reducer::Max,
            Reducer::Mean,
            Reducer::Std,
            Reducer::Rms,
            Reducer::TimeAbove(5.0),
        ] {
            assert!(
                reduced_float(reducer.clone(), &series).is_nan(),
                "{reducer:?}"
            );
        }
    }

    #[test]
    fn a_number_reducer_refuses_a_bool() {
        let flags = [Sample {
            timestamp: at(1.0),
            value: AssertionValue::Bool(true),
        }];
        assert_eq!(
            Reducer::Max.reduce(&flags, at(0.0), at(10.0)),
            Err(ReduceError::NotANumber {
                reducer: "max",
                value_kind: ValueKind::Bool
            })
        );
        assert_eq!(reduce(Reducer::Count, &flags), Some(AssertionValue::Int(1)));
    }

    #[test]
    fn each_reducer_declares_the_unit_of_its_value() {
        let speed = Some(SiUnit::MetersPerSecond);
        assert_eq!(Reducer::Count.si_unit(speed), Some(SiUnit::Dimensionless));
        assert_eq!(Reducer::Count.si_unit(None), Some(SiUnit::Dimensionless));
        assert_eq!(Reducer::MaxGap.si_unit(None), Some(SiUnit::Seconds));
        assert_eq!(
            Reducer::TimeAbove(1.0).si_unit(speed),
            Some(SiUnit::Seconds)
        );
        for reducer in [
            Reducer::Final,
            Reducer::Min,
            Reducer::Max,
            Reducer::Mean,
            Reducer::Std,
            Reducer::Rms,
        ] {
            assert_eq!(reducer.si_unit(speed), speed, "{reducer:?}");
            assert_eq!(reducer.si_unit(None), None, "{reducer:?}");
        }
    }

    #[test]
    fn each_reducer_declares_the_kind_of_its_value() {
        use ValueKind::{Bool, Float, Int, String};
        assert_eq!(Reducer::Final.value_kind(String), Ok(String));
        assert_eq!(Reducer::Count.value_kind(Bool), Ok(Int));
        assert_eq!(Reducer::MaxGap.value_kind(Bool), Ok(Float));
        assert_eq!(Reducer::Max.value_kind(Int), Ok(Int));
        assert_eq!(Reducer::Min.value_kind(Float), Ok(Float));
        for reducer in [
            Reducer::Mean,
            Reducer::Std,
            Reducer::Rms,
            Reducer::TimeAbove(1.0),
        ] {
            assert_eq!(reducer.value_kind(Int), Ok(Float), "{reducer:?}");
        }
        assert_eq!(
            Reducer::Max.value_kind(Bool),
            Err(ReduceError::NotANumber {
                reducer: "max",
                value_kind: Bool
            })
        );
        assert_eq!(
            Reducer::TimeAbove(1.0).value_kind(String),
            Err(ReduceError::NotANumber {
                reducer: "time_above",
                value_kind: String
            })
        );
    }

    #[test]
    fn time_above_holds_each_sample_until_the_next_or_the_end() {
        // Above from 2 to 5 and from 8 to the end at 10.
        let series = [
            float(1.0, 0.0),
            float(2.0, 3.0),
            float(5.0, 0.0),
            float(8.0, 3.0),
        ];
        assert_eq!(reduced_float(Reducer::TimeAbove(1.0), &series), 5.0);
    }

    #[test]
    fn time_above_reads_samples_in_stamp_order() {
        let series = [float(5.0, 0.0), float(2.0, 3.0)];
        assert_eq!(reduced_float(Reducer::TimeAbove(1.0), &series), 3.0);
    }

    #[test]
    fn max_gap_counts_both_edges() {
        // 0 to 1, 1 to 2, 2 to 7, 7 to 10.
        let series = [float(1.0, 0.0), float(2.0, 0.0), float(7.0, 0.0)];
        assert_eq!(reduced_float(Reducer::MaxGap, &series), 5.0);
        assert_eq!(reduced_float(Reducer::MaxGap, &[float(1.0, 0.0)]), 9.0);
        assert_eq!(reduced_float(Reducer::MaxGap, &[float(9.5, 0.0)]), 9.5);
    }

    #[test]
    fn max_gap_holds_an_outside_sample_to_the_edge() {
        let series = [float(-1.0, 0.0), float(12.0, 0.0)];
        assert_eq!(max_gap(&series, at(0.0), at(10.0)), 10.0);
    }
}
