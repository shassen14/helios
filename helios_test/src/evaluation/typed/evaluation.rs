//! [`TypedEvaluation`]: a typed evaluator's evaluation, erased into the
//! [`Evaluator`] the collector feeds. Each role's values are kept typed as
//! they arrive, and at finalize lined up, fitted, computed and read into
//! components, once for every typed kind.

use super::{
    component::Component,
    input::{Inputs, LinedUp},
    kind::TypedEvaluator,
};
use crate::evaluation::{
    derived::{Derived, Refused},
    kind::{Evaluator, InputRejection, Metric},
};

use helios_core::prelude::MonotonicTime;

use std::any::Any;

/// _Per-step._ One entry's evaluation by the typed evaluator `E`: its
/// inputs, the components it offers, and each role's values and refusals.
pub(crate) struct TypedEvaluation<E: TypedEvaluator> {
    evaluator: E,
    inputs: E::Inputs,
    components: Vec<Component<E::Output>>,
    kept: <E::Inputs as Inputs<E>>::Kept,
    /// Each role's refusals, in role order.
    refused: Vec<Refused>,
}

impl<E: TypedEvaluator> TypedEvaluation<E> {
    /// An evaluation by `evaluator` of its `inputs`, nothing kept yet.
    pub(crate) fn new(evaluator: E, inputs: E::Inputs) -> Self {
        Self {
            components: evaluator.components(),
            refused: vec![Refused::default(); inputs.roles().len()],
            kept: Default::default(),
            evaluator,
            inputs,
        }
    }
}

impl<E: TypedEvaluator> Evaluator for TypedEvaluation<E> {
    fn metrics(&self) -> Vec<Metric> {
        self.components.iter().map(Component::metric).collect()
    }

    fn receive(&mut self, role: usize, at: MonotonicTime, payload: &dyn Any) {
        let kept = self
            .inputs
            .keep(&self.evaluator, &mut self.kept, role, at, payload);
        if let (Err(rejection), Some(refused)) = (kept, self.refused.get_mut(role)) {
            refused.add(rejection);
        }
    }

    fn refused(&self) -> &[Refused] {
        &self.refused
    }

    fn derive(&self, until: MonotonicTime, wanted: &[usize]) -> Derived {
        let lined = self.inputs.line_up(&self.kept, until);
        let rows = LinedUp {
            base_samples: lined.base_samples,
            uncovered: lined.uncovered,
            rows: lined
                .rows
                .iter()
                .map(|(at, row)| (*at, <E::Inputs as Inputs<E>>::row(row)))
                .collect(),
        };
        derive(
            rows,
            |run| self.evaluator.fit(run),
            |row, fit| self.evaluator.compute(row, fit),
            &self.components,
            &self.refused,
            wanted,
        )
    }
}

/// What every shape derives from its lined-up `rows`: every covered sample
/// is fitted over once, then each is computed, refusing what can't be;
/// each wanted component is read off what was computed. `inputs` is each
/// role's refusals, in role order.
pub(super) fn derive<R: Copy, F, O>(
    rows: LinedUp<R>,
    fit: impl FnOnce(&[R]) -> Result<F, InputRejection>,
    mut compute: impl FnMut(R, &F) -> Result<O, InputRejection>,
    components: &[Component<O>],
    inputs: &[Refused],
    wanted: &[usize],
) -> Derived {
    let run: Vec<R> = rows.rows.iter().map(|(_, row)| *row).collect();

    let mut not_computed = Refused::default();
    let mut values_refused = vec![Refused::default(); components.len()];
    let metrics = match fit(&run) {
        Ok(fit) => {
            let outputs: Vec<(MonotonicTime, O)> = rows
                .rows
                .iter()
                .filter_map(|(at, row)| match compute(*row, &fit) {
                    Ok(output) => Some((*at, output)),
                    Err(rejection) => {
                        not_computed.add(rejection);
                        None
                    }
                })
                .collect();
            components
                .iter()
                .zip(&mut values_refused)
                .enumerate()
                .map(|(index, (component, refused))| {
                    wanted
                        .contains(&index)
                        .then(|| component.values(&outputs, refused))
                })
                .collect()
        }
        // A run refused whole: every wanted metric has no values, and
        // still carries the refusal to the entries reading it.
        Err(rejection) => {
            run.iter().for_each(|_| not_computed.add(rejection));
            (0..components.len())
                .map(|index| wanted.contains(&index).then(Vec::new))
                .collect()
        }
    };
    Derived::new(
        rows.base_samples,
        rows.uncovered,
        inputs,
        not_computed,
        metrics,
        values_refused,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::AssertionValue,
        collector::{Rejection, Series},
        evaluation::typed::{
            projections::{blend_numbers, number},
            Base, Input,
        },
    };

    use helios_core::kernel::si_unit::SiUnit;

    /// Reads finite `f64`s, kept as they are, the second role held or
    /// interpolated linearly. Fits the sum of every covered base value,
    /// refusing a run with a negative one; a base value of zero is one it
    /// can't compute. Offers the base minus the other role, the fit, and the
    /// base's reciprocal (which refuses a base above 100).
    struct Difference;

    /// One sample: the base, the other role and the fit.
    #[derive(Clone, Copy)]
    struct Values {
        base: f64,
        other: f64,
        sum: f64,
    }

    impl TypedEvaluator for Difference {
        type Inputs = (Base<Self, f64, f64>, Input<Self, f64, f64>);
        type Fit = f64;
        type Output = Values;

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

    /// A base and one role held.
    fn held() -> TypedEvaluation<Difference> {
        let inputs = (
            Base::new("base", |_, value| number(value)),
            Input::held("other", |_, value| number(value)),
        );
        TypedEvaluation::new(Difference, inputs)
    }

    /// A base and one role interpolated linearly.
    fn interpolated() -> TypedEvaluation<Difference> {
        let inputs = (
            Base::new("base", |_, value| number(value)),
            Input::interpolated("other", |_, value| number(value), blend_numbers),
        );
        TypedEvaluation::new(Difference, inputs)
    }

    /// Every metric of `evaluation` up to `until`.
    fn derive_all(evaluation: &TypedEvaluation<Difference>, until: f64) -> Derived {
        evaluation.derive(at(until), &[DIFFERENCE, SUM, RECIPROCAL])
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
        let mut evaluation = held();
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
        let mut evaluation = held();
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
        let mut evaluation = held();
        evaluation.receive(0, at(0.5), &5.0);
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 1);
        assert_eq!((derived.base_samples, derived.uncovered), (2, 1));
    }

    #[test]
    fn values_after_the_run_end_are_not_evaluated() {
        let mut evaluation = held();
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
        let mut evaluation = held();
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
        let mut evaluation = interpolated();
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
        let mut evaluation = held();
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
        let mut evaluation = held();
        evaluation.receive(0, at(0.5), &5.0);
        evaluation.receive(1, at(1.0), &0.0);
        evaluation.receive(0, at(1.0), &-1.0);
        evaluation.receive(0, at(2.0), &12.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            derived.not_computed,
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
        let mut evaluation = held();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &0.0);
        evaluation.receive(0, at(2.0), &12.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            derived.not_computed,
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
        let mut evaluation = held();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);

        let derived = evaluation.derive(at(10.0), &[SUM]);

        assert!(derived.metrics[DIFFERENCE].is_none());
        assert_eq!(
            values(&derived, SUM),
            vec![(1.0, AssertionValue::Float(11.0))]
        );
    }

    #[test]
    fn a_metric_refused_at_one_sample_is_counted_apart_from_the_samples_and_marks_only_its_series()
    {
        let mut evaluation = held();
        evaluation.receive(1, at(1.0), &10.0);
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(2.0), &200.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(series(&derived, RECIPROCAL).samples().len(), 1);
        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 2);
        assert_eq!(derived.not_computed, Refused::default());
        assert_eq!(
            derived.values_refused,
            vec![
                Refused::default(),
                Refused::default(),
                Refused {
                    count: 1,
                    first: Some(InputRejection("a base above 100")),
                },
            ]
        );
        assert_eq!(
            series(&derived, RECIPROCAL).rejection(),
            Some(Rejection::Evaluation(InputRejection("a base above 100")))
        );
        assert_eq!(series(&derived, DIFFERENCE).rejection(), None);
    }
}
