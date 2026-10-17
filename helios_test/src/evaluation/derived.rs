//! An evaluation over a run: what each input keeps as its values arrive, and
//! the derived series computed from that at finalize, one per metric.

use crate::{
    assertion::AssertionValue,
    collector::{Rejection, Sample, Series},
    evaluation::kind::{Alignment, Evaluator, InputRejection, Projection},
};

use helios_core::prelude::MonotonicTime;

use std::any::Any;

/// _Per-step._ One evaluation as the collector feeds it: each role's
/// projections, in arrival order, and what each refused. Its derived series
/// don't exist until [`derive`](Self::derive) computes them.
pub struct RunningEvaluation {
    evaluator: Box<dyn Evaluator>,
    /// Each role's alignment, in role order.
    alignments: Vec<Alignment>,
    /// How many metrics each base sample gives.
    metrics: usize,
    /// Each role's projections, with the stamp of the value each came from.
    kept: Vec<Vec<(MonotonicTime, Projection)>>,
    /// Each role's refusals.
    refused: Vec<Refused>,
}

impl RunningEvaluation {
    /// An evaluation of `evaluator`, its roles aligned as `alignments`
    /// says, deriving `metrics` series.
    pub fn new(evaluator: Box<dyn Evaluator>, alignments: Vec<Alignment>, metrics: usize) -> Self {
        let roles = alignments.len();
        Self {
            evaluator,
            alignments,
            metrics,
            kept: (0..roles).map(|_| Vec::new()).collect(),
            refused: vec![Refused::default(); roles],
        }
    }

    /// A value of `role` stamped `at` arrived, its payload bare: kept as the
    /// evaluator projects it, or counted as refused.
    pub fn receive(&mut self, role: usize, at: MonotonicTime, payload: &dyn Any) {
        let (Some(kept), Some(refused)) = (self.kept.get_mut(role), self.refused.get_mut(role))
        else {
            return;
        };
        match self.evaluator.project(role, payload) {
            Ok(projection) => kept.push((at, projection)),
            Err(rejection) => refused.add(rejection),
        }
    }

    /// Each role's refusals, in role order.
    pub fn refused(&self) -> &[Refused] {
        &self.refused
    }

    /// The derived series, from every value stamped at or before `until`:
    /// the run's last tick, so a value after the run's end is not evaluated,
    /// as it is not judged. Each role's values are taken in stamp order, so
    /// the order they arrived in never matters. Each base value is one
    /// sample of every metric, each held role at its latest value stamped
    /// at or before it; a base value with no held value yet is uncovered.
    /// Each series carries the first refusal, so an assertion on it fails
    /// for it.
    pub fn derive(&self, until: MonotonicTime) -> Derived {
        let ordered: Vec<Vec<&(MonotonicTime, Projection)>> = self
            .kept
            .iter()
            .map(|kept| {
                let mut values: Vec<_> = kept.iter().filter(|(at, _)| at.0 <= until.0).collect();
                values.sort_by(|a, b| a.0 .0.total_cmp(&b.0 .0));
                values
            })
            .collect();
        let rejection = self
            .refused
            .iter()
            .find_map(|refused| refused.first)
            .map(Rejection::Evaluation);

        let mut samples: Vec<Vec<Sample>> = vec![Vec::new(); self.metrics];
        let mut uncovered = 0;
        let base = self
            .alignments
            .iter()
            .position(|alignment| *alignment == Alignment::Base);
        let base_values = base.and_then(|base| ordered.get(base));
        for (at, base_projection) in base_values.into_iter().flatten() {
            let aligned: Option<Vec<&Projection>> = self
                .alignments
                .iter()
                .zip(&ordered)
                .map(|(alignment, values)| match alignment {
                    Alignment::Base => Some(base_projection),
                    Alignment::Hold => values
                        .iter()
                        .take_while(|(held_at, _)| held_at.0 <= at.0)
                        .last()
                        .map(|(_, projection)| projection),
                })
                .collect();
            let Some(aligned) = aligned else {
                uncovered += 1;
                continue;
            };
            for (metric, value) in samples.iter_mut().zip(self.evaluator.compute(&aligned)) {
                metric.push(Sample {
                    timestamp: *at,
                    value: AssertionValue::Float(value),
                });
            }
        }

        Derived {
            base_samples: base_values.map_or(0, Vec::len),
            uncovered,
            metrics: samples
                .into_iter()
                .map(|samples| Series::derived(samples, rejection))
                .collect(),
        }
    }
}

/// _Per-step._ How many values of one role an evaluator refused, and why it
/// refused the first.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Refused {
    pub count: u64,
    pub first: Option<InputRejection>,
}

impl Refused {
    fn add(&mut self, rejection: InputRejection) {
        self.count += 1;
        self.first.get_or_insert(rejection);
    }
}

/// _Per-step._ What one evaluation derived over a run: a series per metric,
/// in the kind's order, and how much of the base it covered.
#[derive(Debug, Clone, PartialEq)]
pub struct Derived {
    pub metrics: Vec<Series>,
    /// Base values stamped within the run.
    pub base_samples: usize,
    /// Base values with no held value yet, which gave no sample.
    pub uncovered: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reads `f64` payloads, kept as they are, and computes base minus held.
    struct Difference;

    impl Evaluator for Difference {
        fn project(&self, _: usize, payload: &dyn Any) -> Result<Projection, InputRejection> {
            match payload.downcast_ref::<f64>() {
                Some(value) if value.is_finite() => Ok(Box::new(*value)),
                _ => Err(InputRejection("not a finite f64")),
            }
        }

        fn compute(&self, aligned: &[&Projection]) -> Vec<f64> {
            let value = |i: usize| *aligned[i].downcast_ref::<f64>().expect("an f64");
            vec![value(0) - value(1)]
        }
    }

    fn running() -> RunningEvaluation {
        RunningEvaluation::new(
            Box::new(Difference),
            vec![Alignment::Base, Alignment::Hold],
            1,
        )
    }

    fn at(t: f64) -> MonotonicTime {
        MonotonicTime(t)
    }

    fn values(series: &Series) -> Vec<(f64, AssertionValue)> {
        series
            .samples()
            .iter()
            .map(|sample| (sample.timestamp.0, sample.value.clone()))
            .collect()
    }

    #[test]
    fn each_base_value_is_judged_against_the_held_value_in_force() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(2.0), &12.0);
        evaluation.receive(1, at(2.5), &20.0);
        evaluation.receive(0, at(3.0), &23.0);

        let derived = evaluation.derive(at(10.0));

        assert_eq!(
            values(&derived.metrics[0]),
            vec![
                (1.0, AssertionValue::Float(1.0)),
                (2.0, AssertionValue::Float(2.0)),
                (3.0, AssertionValue::Float(3.0)),
            ]
        );
        assert_eq!((derived.base_samples, derived.uncovered), (3, 0));
    }

    #[test]
    fn a_held_value_arriving_after_a_later_base_value_still_counts_at_its_stamp() {
        // The reference stamped 1.0 arrives after the base value at 1.0: in
        // stamp order it is in force there.
        let mut evaluation = running();
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(1, at(1.0), &10.0);

        let derived = evaluation.derive(at(10.0));

        assert_eq!(
            values(&derived.metrics[0]),
            vec![(1.0, AssertionValue::Float(1.0))]
        );
    }

    #[test]
    fn a_base_value_before_any_held_value_is_uncovered() {
        let mut evaluation = running();
        evaluation.receive(0, at(0.5), &5.0);
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);

        let derived = evaluation.derive(at(10.0));

        assert_eq!(derived.metrics[0].samples().len(), 1);
        assert_eq!((derived.base_samples, derived.uncovered), (2, 1));
    }

    #[test]
    fn values_after_the_run_end_are_not_evaluated() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(5.0), &15.0);

        let derived = evaluation.derive(at(2.0));

        assert_eq!(
            values(&derived.metrics[0]),
            vec![(1.0, AssertionValue::Float(1.0))]
        );
        assert_eq!(derived.base_samples, 1);
    }

    #[test]
    fn a_refused_value_is_counted_and_marks_every_series() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &f64::NAN);
        evaluation.receive(0, at(2.0), &f64::NAN);
        evaluation.receive(0, at(3.0), &13.0);

        let derived = evaluation.derive(at(10.0));

        assert_eq!(
            evaluation.refused()[0],
            Refused {
                count: 2,
                first: Some(InputRejection("not a finite f64")),
            }
        );
        assert_eq!(evaluation.refused()[1], Refused::default());
        assert_eq!(
            derived.metrics[0].rejection(),
            Some(Rejection::Evaluation(InputRejection("not a finite f64")))
        );
        assert_eq!(derived.metrics[0].samples().len(), 1);
    }
}
