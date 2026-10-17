//! An evaluation over a run: what each input keeps as its values arrive, and
//! the derived series computed from that at finalize, one per metric read.

use crate::{
    assertion::AssertionValue,
    collector::{Rejection, Sample, Series},
    evaluation::kind::{Alignment, Evaluator, InputRejection, Projection, Row},
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
    /// Each role's projections, with the stamp of the value each came from.
    kept: Vec<Vec<(MonotonicTime, Projection)>>,
    /// Each role's refusals.
    refused: Vec<Refused>,
}

impl RunningEvaluation {
    /// An evaluation of `evaluator`, its roles aligned as `alignments`
    /// says.
    pub fn new(evaluator: Box<dyn Evaluator>, alignments: Vec<Alignment>) -> Self {
        let roles = alignments.len();
        Self {
            evaluator,
            alignments,
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

    /// The derived series of the `wanted` metrics, indices into the
    /// evaluator's, from every value stamped at or before `until`: the run's
    /// last tick, so a value after the run's end is not evaluated, as it is
    /// not judged. Each role's values are taken in stamp order, so the order
    /// they arrived in never matters. Each base value is one sample, each
    /// other role lined up at its stamp as its alignment says; a base value
    /// some role has no value for is uncovered. The evaluator grades every
    /// covered sample, refusing what it can't compute, counted like a
    /// refused input. Each series carries the first refusal, so an assertion
    /// on it fails for it. A metric not wanted is not derived.
    pub fn derive(&self, until: MonotonicTime, wanted: &[usize]) -> Derived {
        let ordered: Vec<Vec<&(MonotonicTime, Projection)>> = self
            .kept
            .iter()
            .map(|kept| {
                let mut values: Vec<_> = kept.iter().filter(|(at, _)| at.0 <= until.0).collect();
                values.sort_by(|a, b| a.0 .0.total_cmp(&b.0 .0));
                values
            })
            .collect();

        let base = self
            .alignments
            .iter()
            .position(|alignment| *alignment == Alignment::Base);
        let base_values = base.and_then(|base| ordered.get(base));
        let mut uncovered = 0;
        let mut covered: Vec<(MonotonicTime, Vec<Aligned<'_>>)> = Vec::new();
        for (at, base_projection) in base_values.into_iter().flatten() {
            match self.align(&ordered, *at, base_projection) {
                Some(aligned) => covered.push((*at, aligned)),
                None => uncovered += 1,
            }
        }
        let rows: Vec<Row<'_>> = covered
            .iter()
            .map(|(at, aligned)| Row {
                at: *at,
                aligned: aligned.iter().map(Aligned::projection).collect(),
            })
            .collect();

        let graded = self.evaluator.grade(&rows, wanted, until);
        let rejection = self
            .refused
            .iter()
            .chain([&graded.refused])
            .find_map(|refused| refused.first)
            .map(Rejection::Evaluation);
        Derived {
            base_samples: base_values.map_or(0, Vec::len),
            uncovered,
            refused: graded.refused,
            metrics: graded
                .metrics
                .into_iter()
                .map(|values| {
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
        }
    }

    /// Every role's value at the base value `base` stamped `at`, in role
    /// order, or `None` if some role has none there.
    fn align<'a>(
        &self,
        ordered: &'a [Vec<&'a (MonotonicTime, Projection)>],
        at: MonotonicTime,
        base: &'a Projection,
    ) -> Option<Vec<Aligned<'a>>> {
        self.alignments
            .iter()
            .zip(ordered)
            .enumerate()
            .map(|(role, (alignment, values))| match alignment {
                Alignment::Base => Some(Aligned::Kept(base)),
                Alignment::Hold => values
                    .iter()
                    .take_while(|(held_at, _)| held_at.0 <= at.0)
                    .last()
                    .map(|(_, projection)| Aligned::Kept(projection)),
                Alignment::Interpolate => self.interpolate(role, values, at),
            })
            .collect()
    }

    /// `role`'s value at `at`, from its `values` in stamp order: the value
    /// stamped exactly then, or the evaluator's blend of the two around it.
    /// `None` before the first value or after the last, never extrapolated.
    fn interpolate<'a>(
        &self,
        role: usize,
        values: &[&'a (MonotonicTime, Projection)],
        at: MonotonicTime,
    ) -> Option<Aligned<'a>> {
        let after = values.partition_point(|(stamp, _)| stamp.0 < at.0);
        let (after_at, after_value) = values.get(after)?;
        if after_at.0 == at.0 {
            return Some(Aligned::Kept(after_value));
        }
        let (before_at, before_value) = values.get(after.checked_sub(1)?)?;
        let fraction = (at.0 - before_at.0) / (after_at.0 - before_at.0);
        self.evaluator
            .interpolate(role, before_value, after_value, fraction)
            .map(Aligned::Made)
    }
}

/// _Per-step._ One role's value lined up at a base sample: a kept
/// projection, or one the evaluator blended from two.
enum Aligned<'a> {
    Kept(&'a Projection),
    Made(Projection),
}

impl Aligned<'_> {
    fn projection(&self) -> &Projection {
        match self {
            Aligned::Kept(projection) => projection,
            Aligned::Made(projection) => projection,
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
    /// One more refused, for `rejection`.
    pub(crate) fn add(&mut self, rejection: InputRejection) {
        self.count += 1;
        self.first.get_or_insert(rejection);
    }
}

/// _Per-step._ What one evaluation derived over a run: a series per metric
/// read, how much of the base it covered, and what it couldn't compute.
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
    /// Base values the evaluator couldn't compute, and metrics it couldn't
    /// give at one, and why the first. Any one fails the run, as a refused
    /// input does.
    pub refused: Refused,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::typed::{
        projections::{blend_numbers, number},
        Component, Input, TypedEvaluation, TypedEvaluator,
    };

    use helios_core::kernel::si_unit::SiUnit;

    /// Reads finite `f64`s, kept as they are, the second role held or
    /// interpolated linearly. Fits the sum of every covered base value,
    /// refusing a run with a negative one; a base value of zero is one it
    /// can't compute. Offers the base minus the other role, the fit, the
    /// base's reciprocal (which refuses a base above 100), and the sum of
    /// every difference over the run.
    struct Difference;

    /// One sample: the base, the other role and the fit.
    #[derive(Clone, Copy)]
    struct Values {
        base: f64,
        other: f64,
        sum: f64,
    }

    impl TypedEvaluator for Difference {
        type Inputs = (Input<Self, f64, f64>, Input<Self, f64, f64>);
        type Fit = f64;
        type Error = Values;

        fn components(&self) -> Vec<Component<Values>> {
            vec![
                Component::each("difference", SiUnit::Meters, |v: &Values| {
                    Ok(v.base - v.other)
                }),
                Component::each("sum", SiUnit::Meters, |v: &Values| Ok(v.sum)),
                Component::each("reciprocal", SiUnit::Dimensionless, |v: &Values| {
                    if v.base > 100.0 {
                        Err(InputRejection("a base above 100"))
                    } else {
                        Ok(1.0 / v.base)
                    }
                }),
                Component::whole_run(
                    "total",
                    SiUnit::Meters,
                    |run: &[(MonotonicTime, Values)]| {
                        (!run.is_empty())
                            .then(|| run.iter().map(|(_, v)| v.base - v.other).sum::<f64>())
                    },
                ),
            ]
        }

        fn fit(&self, run: &[(&f64, &f64)]) -> Result<f64, InputRejection> {
            if run.iter().any(|(base, _)| **base < 0.0) {
                return Err(InputRejection("a negative base"));
            }
            Ok(run.iter().map(|(base, _)| **base).sum())
        }

        fn compute(
            &self,
            (base, other): (&f64, &f64),
            sum: &f64,
        ) -> Result<Values, InputRejection> {
            if *base == 0.0 {
                return Err(InputRejection("a zero base"));
            }
            Ok(Values {
                base: *base,
                other: *other,
                sum: *sum,
            })
        }
    }

    const DIFFERENCE: usize = 0;
    const SUM: usize = 1;
    const RECIPROCAL: usize = 2;
    const TOTAL: usize = 3;

    fn running() -> RunningEvaluation {
        running_with(Alignment::Hold)
    }

    /// A base and one role aligned by `alignment`.
    fn running_with(alignment: Alignment) -> RunningEvaluation {
        let other = match alignment {
            Alignment::Interpolate => {
                Input::interpolated("other", |_, value| number(value), blend_numbers)
            }
            _ => Input::held("other", |_, value| number(value)),
        };
        let inputs = (Input::base("base", |_, value| number(value)), other);
        RunningEvaluation::new(
            Box::new(TypedEvaluation::new(Difference, inputs)),
            vec![Alignment::Base, alignment],
        )
    }

    /// Every metric of `evaluation` up to `until`.
    fn derive_all(evaluation: &RunningEvaluation, until: f64) -> Derived {
        evaluation.derive(at(until), &[DIFFERENCE, SUM, RECIPROCAL, TOTAL])
    }

    fn at(t: f64) -> MonotonicTime {
        MonotonicTime(t)
    }

    fn values(derived: &Derived, metric: usize) -> Vec<(f64, AssertionValue)> {
        series(derived, metric)
            .samples()
            .iter()
            .map(|sample| (sample.timestamp.0, sample.value.clone()))
            .collect()
    }

    fn series(derived: &Derived, metric: usize) -> &Series {
        derived.metrics[metric].as_ref().expect("derived")
    }

    #[test]
    fn each_base_value_is_judged_against_the_held_value_in_force() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(2.0), &12.0);
        evaluation.receive(1, at(2.5), &20.0);
        evaluation.receive(0, at(3.0), &23.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            values(&derived, DIFFERENCE),
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

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            values(&derived, DIFFERENCE),
            vec![(1.0, AssertionValue::Float(1.0))]
        );
    }

    #[test]
    fn a_base_value_before_any_held_value_is_uncovered() {
        let mut evaluation = running();
        evaluation.receive(0, at(0.5), &5.0);
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 1);
        assert_eq!((derived.base_samples, derived.uncovered), (2, 1));
    }

    #[test]
    fn values_after_the_run_end_are_not_evaluated() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(5.0), &15.0);

        let derived = derive_all(&evaluation, 2.0);

        assert_eq!(
            values(&derived, DIFFERENCE),
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

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            evaluation.refused()[0],
            Refused {
                count: 2,
                first: Some(InputRejection("a number that isn't finite")),
            }
        );
        assert_eq!(evaluation.refused()[1], Refused::default());
        assert_eq!(
            series(&derived, DIFFERENCE).rejection(),
            Some(Rejection::Evaluation(InputRejection(
                "a number that isn't finite"
            )))
        );
        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 1);
    }

    #[test]
    fn an_interpolated_role_is_blended_at_each_base_stamp_and_never_extrapolated() {
        let mut evaluation = running_with(Alignment::Interpolate);
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(1, at(3.0), &30.0);
        for t in [0.5, 1.0, 2.5, 3.0, 3.5] {
            evaluation.receive(0, at(t), &100.0);
        }

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            values(&derived, DIFFERENCE),
            vec![
                (1.0, AssertionValue::Float(90.0)),
                (2.5, AssertionValue::Float(75.0)),
                (3.0, AssertionValue::Float(70.0)),
            ]
        );
        assert_eq!((derived.base_samples, derived.uncovered), (5, 2));
    }

    #[test]
    fn the_fit_sees_every_covered_sample_before_any_is_computed() {
        let mut evaluation = running();
        evaluation.receive(0, at(0.5), &5.0);
        evaluation.receive(1, at(1.0), &0.0);
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(2.0), &12.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            values(&derived, SUM),
            vec![
                (1.0, AssertionValue::Float(23.0)),
                (2.0, AssertionValue::Float(23.0)),
            ]
        );
    }

    #[test]
    fn a_refused_fit_refuses_every_covered_sample() {
        let mut evaluation = running();
        evaluation.receive(0, at(0.5), &5.0);
        evaluation.receive(1, at(1.0), &0.0);
        evaluation.receive(0, at(1.0), &-1.0);
        evaluation.receive(0, at(2.0), &12.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            derived.refused,
            Refused {
                count: 2,
                first: Some(InputRejection("a negative base")),
            }
        );
        assert_eq!((derived.base_samples, derived.uncovered), (3, 1));
        assert!(series(&derived, DIFFERENCE).samples().is_empty());
        assert_eq!(
            series(&derived, DIFFERENCE).rejection(),
            Some(Rejection::Evaluation(InputRejection("a negative base")))
        );
    }

    #[test]
    fn a_base_value_the_evaluator_cant_compute_is_counted_and_marks_every_series() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &0.0);
        evaluation.receive(0, at(2.0), &12.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            derived.refused,
            Refused {
                count: 1,
                first: Some(InputRejection("a zero base")),
            }
        );
        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 1);
        assert_eq!(
            series(&derived, DIFFERENCE).rejection(),
            Some(Rejection::Evaluation(InputRejection("a zero base")))
        );
    }

    #[test]
    fn only_the_metrics_wanted_are_derived() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);

        let derived = evaluation.derive(at(10.0), &[SUM]);

        assert!(derived.metrics[DIFFERENCE].is_none());
        assert_eq!(
            values(&derived, SUM),
            vec![(1.0, AssertionValue::Float(11.0))]
        );
        assert!(derived.metrics[TOTAL].is_none());
    }

    #[test]
    fn a_whole_run_metric_is_one_sample_stamped_at_the_runs_end() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(2.0), &13.0);

        let derived = derive_all(&evaluation, 10.0);
        let nothing_covered = running().derive(at(10.0), &[TOTAL]);

        assert_eq!(
            values(&derived, TOTAL),
            vec![(10.0, AssertionValue::Float(4.0))]
        );
        assert!(series(&nothing_covered, TOTAL).samples().is_empty());
    }

    #[test]
    fn a_metric_refused_at_one_sample_is_counted_and_marks_every_series() {
        let mut evaluation = running();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(2.0), &200.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(series(&derived, RECIPROCAL).samples().len(), 1);
        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 2);
        assert_eq!(
            derived.refused,
            Refused {
                count: 1,
                first: Some(InputRejection("a base above 100")),
            }
        );
        assert_eq!(
            series(&derived, DIFFERENCE).rejection(),
            Some(Rejection::Evaluation(InputRejection("a base above 100")))
        );
    }
}
