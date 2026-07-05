//! `RunMetrics` — the per-run record of already-reduced control-quality
//! scalars, collected one per run and handed to the cross-run aggregator. The
//! within-run reduction that produces these scalars is done by
//! `crate::statistics`; this module only *stores* the results.

use std::collections::BTreeMap;

/// The string id of one per-run scalar (`"cte.mean"`, `"rise_time"`). Stringly
/// typed for the same reason as the rest of the crate — ids are config-supplied,
/// so the set is open and a typo is caught at load, not compile. A *distinct*
/// type from `statistics::StatId` on purpose: a `MetricId` names *what* was
/// measured, a `StatId` names *how* a series was reduced. Same shape, different
/// meaning — so the compiler won't let one stand in for the other.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MetricId(String);

impl MetricId {
    pub fn new(s: impl Into<String>) -> Self {
        Self(s.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

// TODO: stop gap. this needs to be defined somewhere else
// like in defs.rs or something. maybe think if a different
// solution than this one to define metrics
pub(crate) fn final_pos_e_id() -> MetricId {
    MetricId::new("final_pos_e")
}

// TODO: stop gap. this needs to be defined somewhere else
// like in defs.rs or something. maybe think if a different
// solution than this one to define metrics
pub(crate) fn final_pos_n_id() -> MetricId {
    MetricId::new("final_pos_n")
}

/// _Per/Post-step._ One run's worth of named scalars, plus the metadata
/// identifying which run produced them. Filled as a run finishes, then drained
/// by the cross-run aggregator.
///
/// `seed` and `run_index` are stored *beside* the metric map, never inside it:
/// there is no "mean of the seeds", so keeping them out of `by_metric` makes
/// aggregating them structurally impossible. `BTreeMap` (not `HashMap`) keeps
/// the metrics in a stable, sorted order so the serialized aggregate is
/// deterministic.
///
/// A metric the run didn't measure is simply **absent** from the map — there is
/// no stored "present but undefined" state, because nothing reads a distinction
/// between absent and present-but-undefined (both mean "no number this run").
/// The value is a plain `f64`. If a future need arises for "attempted but
/// undefined" as distinct from "not tracked", reintroduce it as a *named* type,
/// not a nested `Option`.
pub struct RunMetrics {
    by_metric: BTreeMap<MetricId, f64>,
    run_index: u32,
    seed: Option<u64>,
}

impl RunMetrics {
    pub fn new(run_index: u32, seed: Option<u64>) -> Self {
        Self {
            by_metric: BTreeMap::new(),
            run_index,
            seed,
        }
    }

    // Last-writer-wins, like the registries: re-inserting an id overwrites.
    pub fn insert(&mut self, id: MetricId, value: f64) {
        self.by_metric.insert(id, value);
    }

    /// The value `id` recorded this run, or `None` if the run didn't measure it.
    pub fn get(&self, id: &MetricId) -> Option<f64> {
        self.by_metric.get(id).copied()
    }

    /// Walk every recorded `(metric, value)` pair, keys in sorted order — the
    /// cross-run aggregator builds the union of metric ids across runs from
    /// this. Metadata (`seed`, `run_index`) is intentionally unreachable here;
    /// it is never aggregated.
    pub fn iter(&self) -> impl Iterator<Item = (&MetricId, f64)> + '_ {
        self.by_metric.iter().map(|(id, val)| (id, *val))
    }

    pub fn run_index(&self) -> u32 {
        self.run_index
    }

    /// The seed that drove this run, or `None` when the run was unseeded (the
    /// harness left the seed to the scenario file or OS entropy, so the value
    /// is unknown here).
    pub fn seed(&self) -> Option<u64> {
        self.seed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn get_returns_value_when_present_and_none_when_absent() {
        let mut m = RunMetrics::new(0, Some(42));
        m.insert(MetricId::new("cte.mean"), 0.5);

        assert_eq!(m.get(&MetricId::new("cte.mean")), Some(0.5));
        assert_eq!(m.get(&MetricId::new("overshoot")), None);
    }

    #[test]
    fn insert_overrides_previous_value() {
        let mut m = RunMetrics::new(0, Some(0));
        m.insert(MetricId::new("cte.mean"), 1.0);
        m.insert(MetricId::new("cte.mean"), 2.0);
        assert_eq!(m.get(&MetricId::new("cte.mean")), Some(2.0));
    }

    #[test]
    fn iter_yields_metrics_in_sorted_key_order() {
        let mut m = RunMetrics::new(0, Some(0));
        // inserted out of order; BTreeMap must hand them back sorted.
        m.insert(MetricId::new("rise_time"), 1.8);
        m.insert(MetricId::new("cte.mean"), 0.4);
        m.insert(MetricId::new("abs_error"), 0.1);

        let ids: Vec<&str> = m.iter().map(|(id, _)| id.as_str()).collect();
        assert_eq!(ids, ["abs_error", "cte.mean", "rise_time"]);
    }

    #[test]
    fn metadata_is_readable_and_separate_from_metrics() {
        let mut m = RunMetrics::new(7, Some(12345));
        m.insert(MetricId::new("cte.mean"), 0.4);

        assert_eq!(m.run_index(), 7);
        assert_eq!(m.seed(), Some(12345));
        // seed/run_index never leak into the aggregatable metric stream.
        let count = m.iter().count();
        assert_eq!(count, 1, "iter should expose only metrics, not metadata");
    }

    #[test]
    fn an_unseeded_run_reports_no_seed() {
        // A run the harness left unseeded carries `None`, not a stand-in value —
        // the aggregate relies on this to omit it rather than fabricate a seed.
        let m = RunMetrics::new(0, None);
        assert_eq!(m.seed(), None);
    }
}
