//! [`TwoInputs`]: an evaluator kind of two inputs, the base and one other,
//! as its author writes it. [`TwoInputsKind`] registers one, and each entry's
//! evaluation keeps each input's values in a [`RoleBuffer`] and lines the
//! other up at the base's stamps.

use super::{
    buffer::{Lined, LinedUp, Lining, RoleBuffer},
    component::Component,
    derive::derive,
    params,
};
use crate::evaluation::{
    derived::{Derived, Refused},
    kind::{BoundInput, Evaluator, EvaluatorKind, InputRejection, Metric, Role},
};

use helios_core::prelude::MonotonicTime;

use serde::de::DeserializeOwned;

use std::{any::Any, marker::PhantomData};

/// _Pre-step._ An evaluator kind of two inputs: its name, its parameters,
/// each input's name, payload and kept value, the metrics it offers, and
/// what it computes at each sample. The kind is its own evaluator, made by
/// [`bind`](Self::bind) for one entry.
pub trait TwoInputs: Send + Sync + Sized + 'static {
    /// The name a run file's `kind` gives.
    const NAME: &'static str;

    /// The base input's name in `[evaluate.inputs]`: each of its values is
    /// one sample.
    const BASE: &'static str;

    /// The other input's name in `[evaluate.inputs]`, lined up at each base
    /// stamp as [`other_lining`](Self::other_lining) says.
    const OTHER: &'static str;

    /// An entry's `[evaluate.params]`, read strictly: a struct with
    /// `#[serde(deny_unknown_fields)]`, or [`NoParams`](super::NoParams).
    type Params: DeserializeOwned;

    /// The payload type the base's series carries.
    type BasePayload: 'static;

    /// What is kept of each base payload.
    type BaseValue: Send + Sync + 'static;

    /// The payload type the other input's series carries.
    type OtherPayload: 'static;

    /// What is kept of each payload of the other input.
    type OtherValue: Send + Sync + 'static;

    /// What it fits over the whole run before any sample is computed: how
    /// one tree's root sits in another's, say. `()` for none.
    type Fit;

    /// What it computes at one sample: for a comparison, the subject minus
    /// the reference, the subject being what is graded and the reference
    /// what it is judged against.
    type Output;

    /// What is kept of one base payload, or why it is refused.
    fn keep_base(&self, payload: &Self::BasePayload) -> Result<Self::BaseValue, InputRejection>;

    /// What is kept of one payload of the other input, or why it is refused.
    fn keep_other(&self, payload: &Self::OtherPayload) -> Result<Self::OtherValue, InputRejection>;

    /// How the other input lines up at each base stamp.
    fn other_lining(&self) -> Lining<Self::OtherValue>;

    /// The metrics every evaluator of this kind offers, when they are fixed,
    /// so load checks the names a run file reads. `None` when they are known
    /// only once bound, such as one per block of an estimator's state.
    fn menu() -> Option<Vec<Component<Self::Output>>>;

    /// Whether parameters that parsed are ones this kind can evaluate: a
    /// policy not built yet is refused here, naming the case that will need
    /// it.
    fn check(_params: &Self::Params) -> Result<(), String> {
        Ok(())
    }

    /// The evaluator of one entry, from its parameters and the series each
    /// input was bound to, or every reason those series can't be compared.
    fn bind(
        params: Self::Params,
        base: &BoundInput,
        other: &BoundInput,
    ) -> Result<Self, Vec<String>>;

    /// The metrics it offers, each a component read off the output.
    fn components(&self) -> Vec<Component<Self::Output>>;

    /// What it fits from every covered sample, in stamp order, or why
    /// nothing can be, which refuses every sample.
    fn fit(
        &self,
        run: &[(&Self::BaseValue, &Self::OtherValue)],
    ) -> Result<Self::Fit, InputRejection>;

    /// The output at one sample, from the base value, the other input's
    /// value lined up there, and what [`fit`](Self::fit) gave, or why the
    /// two can't be compared.
    fn compute(
        &self,
        base: &Self::BaseValue,
        other: &Self::OtherValue,
        fit: &Self::Fit,
    ) -> Result<Self::Output, InputRejection>;
}

/// _Pre-step._ The two-input kind `K`, registered as any other.
pub struct TwoInputsKind<K>(PhantomData<fn() -> K>);

impl<K> TwoInputsKind<K> {
    pub fn new() -> Self {
        Self(PhantomData)
    }
}

impl<K> Default for TwoInputsKind<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: TwoInputs> EvaluatorKind for TwoInputsKind<K> {
    fn name(&self) -> &'static str {
        K::NAME
    }

    fn roles(&self) -> Vec<Role> {
        vec![
            Role::of::<K::BasePayload>(K::BASE),
            Role::of::<K::OtherPayload>(K::OTHER),
        ]
    }

    fn menu(&self) -> Option<Vec<Metric>> {
        K::menu().map(|components| components.iter().map(Component::metric).collect())
    }

    fn check_params(&self, params: &toml::Table) -> Result<(), String> {
        params::read(K::NAME, params, K::check).map(|_: K::Params| ())
    }

    fn build(
        &self,
        params: &toml::Table,
        inputs: &[BoundInput],
    ) -> Result<Box<dyn Evaluator>, Vec<String>> {
        let params = params::read(K::NAME, params, K::check).map_err(|problem| vec![problem])?;
        let [base, other] = inputs else {
            return Err(vec![format!("{} needs both of its inputs bound", K::NAME)]);
        };
        let evaluation = TwoInputsEvaluation::new(K::bind(params, base, other)?);
        // A fixed menu is the kind's components written a second time, one
        // read before binding and one after, so the types can't make them
        // one list; load checks a run file's names against the first and
        // setup against the second.
        if let Some(menu) = self.menu() {
            let offered = evaluation.metrics();
            if offered != menu {
                return Err(vec![format!(
                    "{} offers {} once bound, but its menu is {}: a kind with a fixed menu \
                     offers exactly its menu",
                    K::NAME,
                    listed(&offered),
                    listed(&menu)
                )]);
            }
        }
        Ok(Box::new(evaluation))
    }
}

/// `metrics` as a run file names them, each with its unit.
fn listed(metrics: &[Metric]) -> String {
    let each: Vec<String> = metrics
        .iter()
        .map(|metric| format!("{} ({})", metric.name, metric.si_unit))
        .collect();
    format!("[{}]", each.join(", "))
}

/// _Per-step._ One entry's evaluation by the two-input kind `K`: the
/// components it offers, and each input's values and refusals.
struct TwoInputsEvaluation<K: TwoInputs> {
    kind: K,
    components: Vec<Component<K::Output>>,
    base: RoleBuffer<K::BaseValue>,
    other: RoleBuffer<K::OtherValue>,
    /// Each input's refusals, the base first.
    refused: [Refused; 2],
}

impl<K: TwoInputs> TwoInputsEvaluation<K> {
    /// An evaluation by `kind`, nothing kept yet.
    fn new(kind: K) -> Self {
        Self {
            components: kind.components(),
            base: RoleBuffer::default(),
            other: RoleBuffer::default(),
            refused: Default::default(),
            kind,
        }
    }

    /// Each base value stamped at or before `until`, and the other input's
    /// value lined up at its stamp; a base value the other has none for is
    /// uncovered.
    fn line_up(&self, until: MonotonicTime) -> LinedUp<(&K::BaseValue, Lined<'_, K::OtherValue>)> {
        let lining = self.kind.other_lining();
        let base = self.base.up_to(until);
        let mut lined = LinedUp {
            base_samples: base.len(),
            uncovered: 0,
            rows: Vec::new(),
        };
        for (at, base) in base {
            match self.other.at(*at, until, &lining) {
                Some(other) => lined.rows.push((*at, (base, other))),
                None => lined.uncovered += 1,
            }
        }
        lined
    }
}

impl<K: TwoInputs> Evaluator for TwoInputsEvaluation<K> {
    fn metrics(&self) -> Vec<Metric> {
        self.components.iter().map(Component::metric).collect()
    }

    fn receive(&mut self, role: usize, at: MonotonicTime, payload: &dyn Any) {
        let kept = match role {
            0 => self
                .base
                .keep(at, payload, |payload| self.kind.keep_base(payload)),
            1 => self
                .other
                .keep(at, payload, |payload| self.kind.keep_other(payload)),
            // Setup feeds only the two roles it bound.
            _ => return,
        };
        if let (Err(rejection), Some(refused)) = (kept, self.refused.get_mut(role)) {
            refused.add(rejection);
        }
    }

    fn refused(&self) -> &[Refused] {
        &self.refused
    }

    fn derive(&self, until: MonotonicTime, wanted: &[usize]) -> Derived {
        let lined = self.line_up(until);
        let rows = LinedUp {
            base_samples: lined.base_samples,
            uncovered: lined.uncovered,
            rows: lined
                .rows
                .iter()
                .map(|(at, (base, other))| (*at, (*base, other.value())))
                .collect(),
        };
        derive(
            rows,
            |run| self.kind.fit(run),
            |(base, other), fit| self.kind.compute(base, other, fit),
            &self.components,
            &self.refused,
            wanted,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::AssertionValue,
        collector::{Rejection, Series},
        evaluation::typed::{
            projections::{blend_numbers, number},
            NoParams,
        },
    };

    use helios_core::kernel::si_unit::SiUnit;
    use helios_core::prelude::AgentId;

    /// Reads finite `f64`s, kept as they are, the other input held or
    /// interpolated linearly. Fits the sum of every covered base value,
    /// refusing a run with a negative one; a base value of zero is one it
    /// can't compute. Offers the base minus the other input, the fit, and
    /// the base's reciprocal (which refuses a base above 100). Bound to a
    /// base of instance [`DRIFTED`], it offers one metric fewer than its menu.
    struct Difference {
        interpolated: bool,
        drifted: bool,
    }

    /// The base instance [`Difference`] binds to offering less than its menu.
    const DRIFTED: &str = "drifted";

    /// One sample: the base, the other input and the fit.
    #[derive(Clone, Copy)]
    struct Values {
        base: f64,
        other: f64,
        sum: f64,
    }

    impl TwoInputs for Difference {
        const NAME: &'static str = "difference";
        const BASE: &'static str = "base";
        const OTHER: &'static str = "other";
        type Params = NoParams;
        type BasePayload = f64;
        type BaseValue = f64;
        type OtherPayload = String;
        type OtherValue = f64;
        type Fit = f64;
        type Output = Values;

        fn keep_base(&self, payload: &f64) -> Result<f64, InputRejection> {
            number(payload)
        }

        fn keep_other(&self, payload: &String) -> Result<f64, InputRejection> {
            payload.parse().map_err(|_| InputRejection("not a number"))
        }

        fn other_lining(&self) -> Lining<f64> {
            if self.interpolated {
                Lining::Interpolate(blend_numbers)
            } else {
                Lining::Hold
            }
        }

        fn menu() -> Option<Vec<Component<Values>>> {
            Some(components())
        }

        fn bind(
            _params: NoParams,
            base: &BoundInput,
            other: &BoundInput,
        ) -> Result<Self, Vec<String>> {
            if base.agent != other.agent {
                return Err(vec!["two agents".to_owned()]);
            }
            Ok(Self {
                interpolated: false,
                drifted: base.instance.as_deref() == Some(DRIFTED),
            })
        }

        fn components(&self) -> Vec<Component<Values>> {
            let mut components = components();
            if self.drifted {
                components.pop();
            }
            components
        }

        fn fit(&self, run: &[(&f64, &f64)]) -> Result<f64, InputRejection> {
            if run.iter().any(|(base, _)| **base < 0.0) {
                return Err(InputRejection("a negative base"));
            }
            Ok(run.iter().map(|(base, _)| **base).sum())
        }

        fn compute(&self, base: &f64, other: &f64, sum: &f64) -> Result<Values, InputRejection> {
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

    fn components() -> Vec<Component<Values>> {
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

    const DIFFERENCE: usize = 0;
    const SUM: usize = 1;
    const RECIPROCAL: usize = 2;

    /// The other input held.
    fn held() -> TwoInputsEvaluation<Difference> {
        TwoInputsEvaluation::new(Difference {
            interpolated: false,
            drifted: false,
        })
    }

    /// The other input interpolated linearly.
    fn interpolated() -> TwoInputsEvaluation<Difference> {
        TwoInputsEvaluation::new(Difference {
            interpolated: true,
            drifted: false,
        })
    }

    /// `value` as the other input sends it.
    fn other(value: f64) -> String {
        value.to_string()
    }

    /// Every metric of `evaluation` up to `until`.
    fn derive_all(evaluation: &TwoInputsEvaluation<Difference>, until: f64) -> Derived {
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

    fn input(agent: &str) -> BoundInput {
        BoundInput {
            agent: AgentId::new(agent),
            instance: None,
            expressed_in: None,
            convention: None,
            rate_hz: None,
            writer: None,
        }
    }

    #[test]
    fn each_input_is_a_role_named_by_its_const_reading_its_payload_the_base_first() {
        let roles = TwoInputsKind::<Difference>::new().roles();

        assert_eq!(
            roles,
            vec![Role::of::<f64>("base"), Role::of::<String>("other")]
        );
    }

    #[test]
    fn the_menu_and_a_bound_evaluations_metrics_are_its_components() {
        let kind = TwoInputsKind::<Difference>::new();
        let evaluation = kind
            .build(&toml::Table::new(), &[input("car"), input("car")])
            .unwrap_or_else(|problems| panic!("{problems:?}"));

        let names: Vec<&str> = evaluation.metrics().iter().map(|m| m.name).collect();

        assert_eq!(kind.menu(), Some(evaluation.metrics()));
        assert_eq!(names, vec!["difference", "sum", "reciprocal"]);
    }

    #[test]
    fn a_bound_evaluation_offering_other_than_the_fixed_menu_is_refused_naming_both() {
        let kind = TwoInputsKind::<Difference>::new();
        let drifted = BoundInput {
            instance: Some(DRIFTED.to_owned()),
            ..input("car")
        };

        let problems = kind
            .build(&toml::Table::new(), &[drifted, input("car")])
            .err()
            .expect("one metric fewer than its menu");

        assert_eq!(
            problems,
            vec![
                "difference offers [difference (m), sum (m)] once bound, but its menu is \
                 [difference (m), sum (m), reciprocal (dimensionless)]: a kind with a fixed menu offers \
                 exactly its menu"
                    .to_owned()
            ]
        );
    }

    #[test]
    fn a_parameter_the_kind_doesnt_read_is_refused_naming_the_kind() {
        let mut params = toml::Table::new();
        params.insert("gain".to_owned(), toml::Value::Float(1.0));
        let kind = TwoInputsKind::<Difference>::new();

        let problem = kind.check_params(&params).expect_err("an unknown key");

        assert!(problem.starts_with("difference: "), "{problem}");
        assert!(kind.check_params(&toml::Table::new()).is_ok());
    }

    #[test]
    fn build_refuses_what_bind_refuses_and_inputs_not_bound() {
        let kind = TwoInputsKind::<Difference>::new();

        let two_agents = kind.build(&toml::Table::new(), &[input("car"), input("truck")]);
        let one_input = kind.build(&toml::Table::new(), &[input("car")]);

        assert_eq!(two_agents.err(), Some(vec!["two agents".to_owned()]));
        assert_eq!(
            one_input.err(),
            Some(vec!["difference needs both of its inputs bound".to_owned()])
        );
    }

    #[test]
    fn each_base_value_is_judged_against_the_held_value_in_force() {
        let mut evaluation = held();
        evaluation.receive(1, at(1.0), &other(10.0));
        evaluation.receive(0, at(1.0), &11.0);
        evaluation.receive(0, at(2.0), &12.0);
        evaluation.receive(1, at(2.5), &other(20.0));
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
        evaluation.receive(1, at(1.0), &other(10.0));

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
        evaluation.receive(1, at(1.0), &other(10.0));
        evaluation.receive(0, at(1.0), &11.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 1);
        assert_eq!((derived.base_samples, derived.uncovered), (2, 1));
    }

    #[test]
    fn values_after_the_run_end_are_not_evaluated() {
        let mut evaluation = held();
        evaluation.receive(1, at(1.0), &other(10.0));
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
    fn a_refused_value_is_counted_against_its_input_and_marks_every_series() {
        let mut evaluation = held();
        evaluation.receive(1, at(1.0), &other(10.0));
        evaluation.receive(1, at(1.5), &"ten".to_owned());
        evaluation.receive(0, at(1.0), &f64::NAN);
        evaluation.receive(0, at(2.0), &f64::NAN);
        evaluation.receive(0, at(3.0), &13.0);

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            evaluation.refused(),
            [
                Refused {
                    count: 2,
                    first: Some(InputRejection("a number that isn't finite")),
                },
                Refused {
                    count: 1,
                    first: Some(InputRejection("not a number")),
                },
            ]
        );
        assert_eq!(
            series(&derived, DIFFERENCE).rejection(),
            Some(Rejection::Evaluation(InputRejection(
                "a number that isn't finite"
            )))
        );
        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 1);
    }

    #[test]
    fn an_interpolated_input_is_blended_at_each_base_stamp_and_never_extrapolated() {
        let mut evaluation = interpolated();
        evaluation.receive(1, at(1.0), &other(10.0));
        evaluation.receive(1, at(3.0), &other(30.0));
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
        evaluation.receive(1, at(1.0), &other(0.0));
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
        evaluation.receive(1, at(1.0), &other(0.0));
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
    fn a_base_value_the_kind_cant_compute_is_counted_and_marks_every_series() {
        let mut evaluation = held();
        evaluation.receive(1, at(1.0), &other(10.0));
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
        evaluation.receive(1, at(1.0), &other(10.0));
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
        evaluation.receive(1, at(1.0), &other(10.0));
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
