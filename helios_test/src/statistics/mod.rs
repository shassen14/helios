//! Named, pure reducers over `&[f64]` — the one registry that serves both
//! levels of a Monte Carlo run.
//!
//! There is a single asymmetry that shapes this module: *within-run* reduction
//! is optional and metric-specific (a `cte(t)` series collapses to `cte.mean`),
//! while *cross-run* aggregation is universal (N finished runs are just a bag of
//! N numbers). Both ends are "reduce a slice of `f64` to one `f64`," so the same
//! `StatisticTable` does both jobs — one `mean`, two levels.
//!
//! Mirrors `assertion::extract`: function-pointer reducers (not `Fn`),
//! last-writer-wins `register`, a `Default`/`new` empty table, and a
//! `standard_statistics()` constructor for the built-in set.

use std::collections::HashMap;

/// A pure reducer: a bag of samples in, one scalar out (or `None` when the
/// reduction is undefined, e.g. an empty slice).
// `fn` (not `Fn`) means only captureless functions/closures coerce in. The
// table stores plain function pointers, so registrations are cheap to copy and
// allocate nothing. Switch to `Box<dyn Fn(...)>` only if a reducer ever needs to
// close over state; until then every statistic is a pure projection.
pub type Statistic = fn(&[f64]) -> Option<f64>;

/// The string id a `Statistic` is registered and looked up under (`"mean"`,
/// `"std"`, …). Stringly-typed on purpose: ids come from config, so the set is
/// open (third parties `register` their own) and a typo is caught at load by
/// validating against the table, not at compile time.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct StatId(String);

impl StatId {
    pub fn new(s: impl Into<String>) -> Self {
        Self(s.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// A registry of named reducers. Built once (usually from
/// [`standard_statistics`]), then queried by id to reduce a slice — within a
/// single run *or* across runs.
#[derive(Default)]
pub struct StatisticTable {
    by_stat: HashMap<StatId, Statistic>,
}


impl StatisticTable {
    pub fn new() -> Self {
        Self::default()
    }

    // Last-writer-wins: registering the same id twice silently overwrites.
    // Intentional — lets a test install a fixture reducer over the standard set
    // without a separate "override" entry point.
    pub fn register(&mut self, stat_id: StatId, stat: Statistic) {
        self.by_stat.insert(stat_id, stat);
    }

    /// Look up `stat_id` and run it over `data`. Returns `None` for two cases
    /// collapsed for now: (1) no reducer registered under `stat_id`, (2) the
    /// reducer itself returned `None` (e.g. `data` was empty). Split into a
    /// richer error if a caller ever needs to tell them apart.
    pub fn apply(&self, stat_id: &StatId, data: &[f64]) -> Option<f64> {
        let stat = self.by_stat.get(stat_id)?;
        stat(data)
    }
}

/// The built-in reducer set: `mean`, `std`, `min`, `max`. Add `p95` / `variance`
/// here as one `register` line each when a metric needs them.
pub fn standard_statistics() -> StatisticTable {
    let mut table = StatisticTable::new();

    table.register(StatId::new("mean"), mean);
    table.register(StatId::new("std"), std);
    table.register(StatId::new("min"), min);
    table.register(StatId::new("max"), max);

    table
}

/// Smallest sample. `None` for an empty slice. NaN-tolerant: `f64::min` returns
/// the non-NaN argument, so a stray NaN won't swallow the result.
fn min(data: &[f64]) -> Option<f64> {
    data.iter().copied().reduce(f64::min)
}

/// Largest sample. `None` for an empty slice. NaN-tolerant (see [`min`]).
fn max(data: &[f64]) -> Option<f64> {
    data.iter().copied().reduce(f64::max)
}

/// Arithmetic mean. `None` for an empty slice (guards the divide-by-zero).
fn mean(data: &[f64]) -> Option<f64> {
    if data.is_empty() {
        return None;
    }
    Some(data.iter().sum::<f64>() / data.len() as f64)
}

/// Population standard deviation (÷N, not ÷N−1): the spread of *these* N runs,
/// not an estimate of a wider population. `None` for an empty slice.
fn std(data: &[f64]) -> Option<f64> {
    if data.is_empty() {
        return None;
    }

    let n = data.len() as f64;
    let m = mean(data)?;
    let var = data.iter().map(|x| (x - m).powi(2)).sum::<f64>() / n;
    Some(var.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Comparing reduced floats: the inputs below are chosen so results are
    // exactly representable, but `std` goes through a `sqrt`, so use a tolerance
    // for it rather than `assert_eq!`.
    const EPS: f64 = 1e-9;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn mean_of_known_slice() {
        let table = standard_statistics();
        let got = table.apply(&StatId::new("mean"), &[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(got, Some(2.5));
    }

    #[test]
    fn min_and_max_of_known_slice() {
        let table = standard_statistics();
        let data = [3.0, 1.0, 4.0, 1.0, 5.0];
        assert_eq!(table.apply(&StatId::new("min"), &data), Some(1.0));
        assert_eq!(table.apply(&StatId::new("max"), &data), Some(5.0));
    }

    #[test]
    fn population_std_of_known_slice() {
        // Classic worked example: mean 5, population variance 4, std 2.
        let table = standard_statistics();
        let data = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let got = table.apply(&StatId::new("std"), &data).unwrap();
        assert!(approx(got, 2.0), "expected ~2.0, got {got}");
    }

    #[test]
    fn empty_slice_reduces_to_none() {
        let table = standard_statistics();
        for id in ["mean", "std", "min", "max"] {
            assert_eq!(
                table.apply(&StatId::new(id), &[]),
                None,
                "{id} of empty slice should be None"
            );
        }
    }

    #[test]
    fn unknown_id_returns_none() {
        let table = standard_statistics();
        assert_eq!(table.apply(&StatId::new("median"), &[1.0, 2.0]), None);
    }

    #[test]
    fn register_overrides_previous_entry() {
        let mut table = standard_statistics();
        table.register(StatId::new("mean"), |_| Some(99.0));
        assert_eq!(
            table.apply(&StatId::new("mean"), &[1.0, 2.0, 3.0]),
            Some(99.0)
        );
    }

    #[test]
    fn default_table_is_empty() {
        let table = StatisticTable::default();
        assert_eq!(table.apply(&StatId::new("mean"), &[1.0, 2.0]), None);
    }
}
