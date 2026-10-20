//! What an evaluation derived over a run: a series per metric read, how much
//! of its base the run covered, and what it refused.

use crate::{
    assertion::AssertionValue,
    collector::{Rejection, Sample, Series},
    evaluation::kind::InputRejection,
};

use helios_core::prelude::MonotonicTime;

/// _Per-step._ What one evaluation derived over a run: a series per metric
/// read, how much of the base it covered, and what it couldn't compute.
/// Every refusal fails the run, as a refused input does.
#[derive(Debug, Clone, PartialEq)]
pub struct Derived {
    /// One per metric, in the evaluator's order: `None` for one nothing
    /// reads, which is not derived.
    pub metrics: Vec<Option<Series>>,
    /// Base values stamped within the run.
    pub base_samples: usize,
    /// Base values some role had no value for, which gave no sample: before
    /// the first held value, or outside an interpolated role's values.
    pub uncovered: usize,
    /// Covered base values the evaluator couldn't compute, each a sample no
    /// metric has, and why the first.
    pub not_computed: Refused,
    /// One per metric, in the evaluator's order: the computed samples it
    /// couldn't give a value at (a heading of a pose facing straight up),
    /// and why the first.
    pub values_refused: Vec<Refused>,
}

impl Derived {
    /// What an evaluation derived: `metrics`, each wanted metric's values in
    /// stamp order and `None` for one not wanted, with what the evaluation
    /// refused. Each series carries the first refusal its values lost to, an
    /// input's (`inputs`, in role order) before a sample's before its own
    /// metric's, so an assertion on it fails for that. Another metric's
    /// refused value leaves it whole.
    pub fn new(
        base_samples: usize,
        uncovered: usize,
        inputs: &[Refused],
        not_computed: Refused,
        metrics: Vec<Option<Vec<(MonotonicTime, f64)>>>,
        values_refused: Vec<Refused>,
    ) -> Self {
        let shared = inputs
            .iter()
            .chain([&not_computed])
            .find_map(|refused| refused.first);
        Self {
            metrics: metrics
                .into_iter()
                .enumerate()
                .map(|(metric, values)| {
                    let own = values_refused.get(metric).and_then(|refused| refused.first);
                    let rejection = shared.or(own).map(Rejection::Evaluation);
                    values.map(|values| {
                        let samples = values
                            .into_iter()
                            .map(|(timestamp, value)| Sample {
                                timestamp,
                                value: AssertionValue::Float(value),
                            })
                            .collect();
                        Series::derived(samples, rejection)
                    })
                })
                .collect(),
            base_samples,
            uncovered,
            not_computed,
            values_refused,
        }
    }
}

/// _Per-step._ How many values an evaluation refused, of one role or one
/// metric or as samples, and why it refused the first.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Refused {
    pub count: u64,
    pub first: Option<InputRejection>,
}

impl Refused {
    /// One more refused, for `rejection`.
    pub(crate) fn add(&mut self, rejection: InputRejection) {
        self.count += 1;
        self.first.get_or_insert(rejection);
    }
}
