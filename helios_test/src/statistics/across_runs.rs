//! Cross-run statistics: one entry's value from each run of a batch, reduced
//! to one number. A measure's values are numbers; an assertion's are
//! verdicts, which only `pass_rate` reads.

use super::numbers::{max, mean, min, percentile, sample_std};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use std::fmt;

/// _Shared (pre + post)._ How a batch reduces one entry's per-run values.
/// Written `"mean"`, or `{ percentile = 95.0 }` for the one with a
/// parameter. A closed set, as [`Reducer`](super::within_run::Reducer) is:
/// a statistic is math, not a payload type, and a typo is refused at parse.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CrossRun {
    /// The mean of the runs' numbers.
    Mean,
    /// Their sample standard deviation (divided by one less than the run
    /// count): the runs are a sample of every seed, so their spread is an
    /// estimate. None for fewer than two runs.
    Std,
    /// The smallest.
    Min,
    /// The largest.
    Max,
    /// The value that percent of the way through them, sorted.
    Percentile(Percent),
    /// The fraction of runs whose verdict was a pass, from 0 to 1.
    PassRate,
}

impl CrossRun {
    /// The reduction of `numbers`, one per run, or `None` when there are
    /// none; refused for `pass_rate`, which reads verdicts.
    pub fn of_numbers(&self, numbers: &[f64]) -> Result<Option<f64>, CrossRunError> {
        Ok(match self {
            CrossRun::Mean => mean(numbers),
            CrossRun::Std => sample_std(numbers),
            CrossRun::Min => min(numbers),
            CrossRun::Max => max(numbers),
            CrossRun::Percentile(percent) => percentile(numbers, percent.0),
            CrossRun::PassRate => return Err(CrossRunError::NeedsVerdicts(*self)),
        })
    }

    /// The reduction of `passed`, each run's verdict, or `None` when there
    /// are none; refused for every statistic but `pass_rate`, which reads
    /// numbers.
    pub fn of_verdicts(&self, passed: &[bool]) -> Result<Option<f64>, CrossRunError> {
        match self {
            CrossRun::PassRate => {
                let passes = passed.iter().filter(|passed| **passed).count();
                Ok((!passed.is_empty()).then(|| passes as f64 / passed.len() as f64))
            }
            _ => Err(CrossRunError::NeedsNumbers(*self)),
        }
    }
}

/// As a report keys it: its name, a percentile as `p` and its percent.
impl fmt::Display for CrossRun {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CrossRun::Mean => f.write_str("mean"),
            CrossRun::Std => f.write_str("std"),
            CrossRun::Min => f.write_str("min"),
            CrossRun::Max => f.write_str("max"),
            CrossRun::Percentile(percent) => write!(f, "p{}", percent.0),
            CrossRun::PassRate => f.write_str("pass_rate"),
        }
    }
}

/// _Shared (pre + post)._ A percent, finite and from 0 to 100, refused at
/// parse otherwise.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
#[serde(try_from = "f64", into = "f64")]
pub struct Percent(f64);

impl TryFrom<f64> for Percent {
    type Error = String;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        if (0.0..=100.0).contains(&value) {
            Ok(Self(value))
        } else {
            Err(format!("a percentile is from 0 to 100, not {value}"))
        }
    }
}

impl From<Percent> for f64 {
    fn from(value: Percent) -> Self {
        value.0
    }
}

/// _Pre-step._ A statistic asked of values it doesn't read.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum CrossRunError {
    #[error("{0} reads verdicts, not numbers")]
    NeedsVerdicts(CrossRun),
    #[error("{0} reads numbers, not verdicts")]
    NeedsNumbers(CrossRun),
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One statistic as a run file would spell it, under the key `of`.
    #[derive(Debug, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Spelled {
        of: CrossRun,
    }

    fn parse(text: &str) -> Result<CrossRun, toml::de::Error> {
        toml::from_str::<Spelled>(text).map(|spelled| spelled.of)
    }

    #[test]
    fn a_statistic_parses_as_a_string_or_a_percentile_table() {
        assert_eq!(parse(r#"of = "mean""#).ok(), Some(CrossRun::Mean));
        assert_eq!(parse(r#"of = "pass_rate""#).ok(), Some(CrossRun::PassRate));
        assert_eq!(
            parse("of = { percentile = 95.0 }").ok(),
            Some(CrossRun::Percentile(Percent(95.0)))
        );
    }

    #[test]
    fn a_typo_and_a_percent_out_of_range_are_refused_at_parse() {
        assert!(parse(r#"of = "median""#).is_err());
        assert!(parse("of = { percentile = 101.0 }").is_err());
        assert!(parse("of = { percentile = -1.0 }").is_err());
        assert!(parse("of = { percentile = nan }").is_err());
    }

    #[test]
    fn numbers_statistics_read_numbers_and_pass_rate_reads_verdicts() {
        let numbers = [1.0, 2.0, 3.0, 4.0];

        assert_eq!(CrossRun::Mean.of_numbers(&numbers), Ok(Some(2.5)));
        assert_eq!(
            CrossRun::Percentile(Percent(50.0)).of_numbers(&numbers),
            Ok(Some(2.5))
        );
        assert_eq!(
            CrossRun::PassRate.of_verdicts(&[true, false, true, true]),
            Ok(Some(0.75))
        );
        assert_eq!(CrossRun::PassRate.of_verdicts(&[]), Ok(None));
    }

    #[test]
    fn a_statistic_refuses_the_other_kind_of_value() {
        assert_eq!(
            CrossRun::PassRate.of_numbers(&[1.0]),
            Err(CrossRunError::NeedsVerdicts(CrossRun::PassRate))
        );
        assert_eq!(
            CrossRun::Max.of_verdicts(&[true]),
            Err(CrossRunError::NeedsNumbers(CrossRun::Max))
        );
    }

    #[test]
    fn each_statistic_is_keyed_by_its_name() {
        let keys: Vec<String> = [
            CrossRun::Mean,
            CrossRun::Std,
            CrossRun::Min,
            CrossRun::Max,
            CrossRun::Percentile(Percent(99.5)),
            CrossRun::PassRate,
        ]
        .iter()
        .map(ToString::to_string)
        .collect();

        assert_eq!(keys, ["mean", "std", "min", "max", "p99.5", "pass_rate"]);
    }
}
