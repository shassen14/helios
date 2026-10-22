//! [`TwoInputs`]: an evaluator kind of two inputs, the base and one other,
//! as its author writes it. [`TwoInputsKind`] registers one, and each entry's
//! evaluation keeps each input's values in a [`RoleBuffer`] and lines the
//! other up at the base's stamps.

use super::{
    buffer::{GapRule, Lined, LinedUp, Lining, RoleBuffer, Row, Unlined},
    component::Component,
    derive::derive,
    params,
};
use crate::evaluation::{
    config::Gap,
    derived::{Coverage, Derived, Refused, Uncovered, UncoveredCounts},
    kind::{BoundInput, Cap, CapUse, Evaluator, EvaluatorKind, InputRejection, Metric, Role},
};

use helios_core::prelude::MonotonicTime;

use serde::de::DeserializeOwned;

use std::{any::Any, marker::PhantomData};

/// _Pre-step._ An evaluator kind of two inputs: its name, its parameters,
/// each input's name, payload and kept value, the metrics it offers, and
/// what it computes at each sample, sample by sample in stamp order. The kind is its own evaluator, made by
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

    /// What it fits over the whole run before any sample is computed, for a
    /// policy that needs every sample: a best fit of one tree's root in
    /// another's, say. `()` for none.
    type Fit;

    /// What it carries from one sample to the next, starting from its
    /// default: the first sample's poses, say. `()` for none. Kept by the
    /// evaluation between ticks, so it is as shareable as the evaluation.
    type State: Default + Send + Sync;

    /// What it computes at one sample: for a comparison, the subject minus
    /// the reference, the subject being what is graded and the reference
    /// what it is judged against.
    type Output;

    /// What is kept of one base payload, or why it is refused.
    fn keep_base(&self, payload: &Self::BasePayload) -> Result<Self::BaseValue, InputRejection>;

    /// What is kept of one payload of the other input, or why it is refused.
    fn keep_other(&self, payload: &Self::OtherPayload) -> Result<Self::OtherValue, InputRejection>;

    /// How the other input lines up at each base stamp, and whether across
    /// gaps up to a cap, which the run gives.
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
    /// value lined up there, the sample's facts, the state the samples
    /// before it left, and what [`fit`](Self::fit) gave, or why the two
    /// can't be compared. A refused sample still updates the state.
    fn compute(
        &self,
        base: &Self::BaseValue,
        other: &Self::OtherValue,
        row: RowFacts,
        state: &mut Self::State,
        fit: &Self::Fit,
    ) -> Result<Self::Output, InputRejection>;
}

/// _Per-step._ What [`TwoInputs::compute`] is told of one sample besides
/// its values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RowFacts {
    /// The base value's stamp.
    pub at: MonotonicTime,
    /// Whether a gap comes before it: in the base, wider than its cap, or a
    /// base value since the sample before that gave none, because the other
    /// input was in a gap there or couldn't be blended.
    pub follows_gap: bool,
    /// For an other input held, whether its value here is newer than the
    /// sample before; `None` for one interpolated, blended at every sample.
    pub other_fresh: Option<bool>,
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
        gaps: &[Option<Gap>],
    ) -> Result<Box<dyn Evaluator>, Vec<String>> {
        let params = params::read(K::NAME, params, K::check).map_err(|problem| vec![problem])?;
        let ([base, other], [base_gap, other_gap]) = (inputs, gaps) else {
            return Err(vec![format!("{} needs both of its inputs bound", K::NAME)]);
        };
        let kind = K::bind(params, base, other)?;
        let base_cap = Cap::of(K::BASE, base, *base_gap, CapUse::FindsGaps);
        let other_rule = kind.other_lining().with_cap(K::OTHER, other, *other_gap);
        let (base_cap, other_rule) = match (base_cap, other_rule) {
            (Ok(base_cap), Ok(other_rule)) => (base_cap, other_rule),
            (base_cap, other_rule) => {
                return Err(base_cap.err().into_iter().chain(other_rule.err()).collect())
            }
        };
        let evaluation = TwoInputsEvaluation::new(kind, base_cap, other_rule);
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
/// components it offers, the cap the base's gaps are found against, how the
/// other input lines up, and each input's values and refusals.
struct TwoInputsEvaluation<K: TwoInputs> {
    kind: K,
    components: Vec<Component<K::Output>>,
    base_cap: Cap,
    other_rule: GapRule<K::OtherValue>,
    base: RoleBuffer<K::BaseValue>,
    other: RoleBuffer<K::OtherValue>,
    /// Each input's refusals, the base first.
    refused: [Refused; 2],
}

impl<K: TwoInputs> TwoInputsEvaluation<K> {
    /// An evaluation by `kind`, the base's gaps found against `base_cap`,
    /// the other input lined up by `other_rule`, nothing kept yet.
    fn new(kind: K, base_cap: Cap, other_rule: GapRule<K::OtherValue>) -> Self {
        Self {
            components: kind.components(),
            base_cap,
            other_rule,
            base: RoleBuffer::default(),
            other: RoleBuffer::default(),
            refused: Default::default(),
            kind,
        }
    }

    /// Each base value stamped at or before `until`, and the other input's
    /// value lined up at its stamp; a base value the other has none for is
    /// uncovered, and one between two values it can't blend is refused. A
    /// row follows a gap when the base value before it is further back than
    /// the base's cap, or a base value since the row before gave no row
    /// because the other input was in a gap or couldn't be blended there.
    fn line_up(&self, until: MonotonicTime) -> LinedUp<(&K::BaseValue, Lined<'_, K::OtherValue>)> {
        let base = self.base.up_to(until);
        let mut base_gaps = 0;
        let mut other_uncovered = UncoveredCounts::default();
        let mut unblended = Refused::default();
        let mut rows = Vec::new();
        let mut previous: Option<MonotonicTime> = None;
        let mut gap_since_last_row = false;
        for (at, base) in base {
            if previous.is_some_and(|previous| !self.base_cap.spans(at.0 - previous.0)) {
                base_gaps += 1;
                gap_since_last_row = true;
            }
            previous = Some(*at);
            match self.other.at(*at, until, &self.other_rule) {
                Ok(other) => {
                    rows.push(Row {
                        at: *at,
                        follows_gap: gap_since_last_row,
                        values: (base, other),
                    });
                    gap_since_last_row = false;
                }
                Err(Unlined::Uncovered(uncovered)) => {
                    gap_since_last_row |= uncovered == Uncovered::InGap;
                    other_uncovered.add(uncovered);
                }
                Err(Unlined::Refused(rejection)) => {
                    gap_since_last_row = true;
                    unblended.add(rejection);
                }
            }
        }
        LinedUp {
            base_samples: base.len(),
            uncovered: other_uncovered.total(),
            coverage: vec![
                Coverage::Base { gaps: base_gaps },
                Coverage::Lined {
                    uncovered: other_uncovered,
                },
            ],
            unblended,
            rows,
        }
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

    fn caps(&self) -> Vec<Option<Cap>> {
        vec![Some(self.base_cap), self.other_rule.cap()]
    }

    fn refused(&self) -> &[Refused] {
        &self.refused
    }

    fn derive(&self, until: MonotonicTime, wanted: &[usize]) -> Derived {
        let lined = self.line_up(until);
        let rows = LinedUp {
            base_samples: lined.base_samples,
            uncovered: lined.uncovered,
            coverage: lined.coverage,
            unblended: lined.unblended,
            rows: as_computed(&lined.rows),
        };
        let mut state = K::State::default();
        derive(
            rows,
            |run| {
                let run: Vec<_> = run.iter().map(|(_, base, other)| (*base, *other)).collect();
                self.kind.fit(&run)
            },
            |(row, base, other), fit| self.kind.compute(base, other, row, &mut state, fit),
            &self.components,
            &self.refused,
            wanted,
        )
    }
}

/// Each lined-up row as [`TwoInputs::compute`] is given it: its facts, the
/// base value and the other input's. A held value is fresh at the first row
/// stamped after the row before.
fn as_computed<'a, B, O>(
    rows: &'a [Row<(&'a B, Lined<'a, O>)>],
) -> Vec<Row<(RowFacts, &'a B, &'a O)>> {
    let mut previous: Option<MonotonicTime> = None;
    rows.iter()
        .map(|row| {
            let (base, other) = &row.values;
            let other_fresh = other
                .held_since()
                .map(|since| previous.is_none_or(|previous| since.0 > previous.0));
            previous = Some(row.at);
            let facts = RowFacts {
                at: row.at,
                follows_gap: row.follows_gap,
                other_fresh,
            };
            Row {
                at: row.at,
                follows_gap: row.follows_gap,
                values: (facts, *base, other.value()),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::AssertionValue,
        collector::{Rejection, Series},
        evaluation::{
            derived::UncoveredCounts,
            typed::{
                projections::{blend_numbers, number},
                NoParams,
            },
            GapWidth,
        },
    };

    use helios_core::kernel::si_unit::SiUnit;
    use helios_core::prelude::AgentId;

    /// Reads finite `f64`s, kept as they are, the other input held
    /// indefinitely, or interpolated linearly within its cap when bound to
    /// instance [`INTERPOLATED`]. Fits the sum of every covered base value,
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

    /// The other instance [`Difference`] binds to interpolating it.
    const INTERPOLATED: &str = "interpolated";

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
        type State = ();
        type Output = Values;

        fn keep_base(&self, payload: &f64) -> Result<f64, InputRejection> {
            number(payload)
        }

        fn keep_other(&self, payload: &String) -> Result<f64, InputRejection> {
            payload.parse().map_err(|_| InputRejection("not a number"))
        }

        fn other_lining(&self) -> Lining<f64> {
            if self.interpolated {
                Lining::InterpolateWithinCap(blend_numbers)
            } else {
                Lining::HoldIndefinitely
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
                interpolated: other.instance.as_deref() == Some(INTERPOLATED),
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

        fn compute(
            &self,
            base: &f64,
            other: &f64,
            _row: RowFacts,
            _state: &mut (),
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

    /// The other input held however old, the base's gaps found past 10 s.
    fn held() -> TwoInputsEvaluation<Difference> {
        TwoInputsEvaluation::new(
            Difference {
                interpolated: false,
                drifted: false,
            },
            Cap::in_seconds(width(10.0)),
            GapRule::Hold(None),
        )
    }

    /// The other input interpolated linearly across gaps up to 10 s, the
    /// base's gaps found past 10 s.
    fn interpolated() -> TwoInputsEvaluation<Difference> {
        TwoInputsEvaluation::new(
            Difference {
                interpolated: true,
                drifted: false,
            },
            Cap::in_seconds(width(10.0)),
            GapRule::Interpolate(blend_numbers, Cap::in_seconds(width(10.0))),
        )
    }

    fn width(value: f64) -> GapWidth {
        GapWidth::try_from(value).expect("positive and finite")
    }

    /// The base's gaps found past 1 s.
    fn base_gap() -> Option<Gap> {
        Some(Gap::Seconds(width(1.0)))
    }

    /// Where the base values the other input had none for fell.
    fn other_uncovered(derived: &Derived) -> UncoveredCounts {
        match derived.coverage[..] {
            [_, Coverage::Lined { uncovered }] => uncovered,
            _ => panic!("the base's then the other's: {:?}", derived.coverage),
        }
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

    fn before_first(count: usize) -> UncoveredCounts {
        UncoveredCounts {
            before_first: count,
            ..UncoveredCounts::default()
        }
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
            .build(
                &toml::Table::new(),
                &[input("car"), input("car")],
                &[base_gap(), None],
            )
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
            .build(
                &toml::Table::new(),
                &[drifted, input("car")],
                &[base_gap(), None],
            )
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

        let two_agents = kind.build(
            &toml::Table::new(),
            &[input("car"), input("truck")],
            &[base_gap(), None],
        );
        let one_input = kind.build(&toml::Table::new(), &[input("car")], &[base_gap(), None]);

        assert_eq!(two_agents.err(), Some(vec!["two agents".to_owned()]));
        assert_eq!(
            one_input.err(),
            Some(vec!["difference needs both of its inputs bound".to_owned()])
        );
    }

    /// The other input, interpolated, at `rate_hz`.
    fn interpolated_input(rate_hz: Option<f64>) -> BoundInput {
        BoundInput {
            instance: Some(INTERPOLATED.to_owned()),
            rate_hz,
            ..input("car")
        }
    }

    #[test]
    fn a_capped_role_takes_a_multiple_of_its_period_or_the_seconds_given() {
        let kind = TwoInputsKind::<Difference>::new();
        let caps = |gap| {
            kind.build(
                &toml::Table::new(),
                &[input("car"), interpolated_input(Some(10.0))],
                &[base_gap(), Some(gap)],
            )
            .map(|evaluation| evaluation.caps())
            .unwrap_or_else(|problems| panic!("{problems:?}"))
        };
        let seconds_and_multiple = |caps: Vec<Option<Cap>>| {
            caps.into_iter()
                .map(|cap| cap.map(|cap| (cap.seconds(), cap.multiple())))
                .collect::<Vec<_>>()
        };

        assert_eq!(
            seconds_and_multiple(caps(Gap::Multiple(width(3.0)))),
            vec![Some((1.0, None)), Some((0.3, Some(3.0)))]
        );
        assert_eq!(
            seconds_and_multiple(caps(Gap::Seconds(width(0.5)))),
            vec![Some((1.0, None)), Some((0.5, None))]
        );
    }

    #[test]
    fn a_role_held_indefinitely_needs_no_cap() {
        let kind = TwoInputsKind::<Difference>::new();

        let evaluation = kind
            .build(
                &toml::Table::new(),
                &[input("car"), input("car")],
                &[base_gap(), None],
            )
            .unwrap_or_else(|problems| panic!("{problems:?}"));

        assert_eq!(
            evaluation.caps(),
            vec![Some(Cap::in_seconds(width(1.0))), None]
        );
    }

    #[test]
    fn a_capped_role_with_no_cap_to_take_is_refused_naming_the_role_and_the_remedies() {
        let kind = TwoInputsKind::<Difference>::new();
        let problems = |rate_hz, gap| {
            kind.build(
                &toml::Table::new(),
                &[input("car"), interpolated_input(rate_hz)],
                &[base_gap(), gap],
            )
            .err()
            .expect("no cap")
        };

        assert_eq!(
            problems(None, Some(Gap::Multiple(width(3.0)))),
            vec![
                "the other spans gaps up to a cap, and its series declares no rate to take a \
                 multiple of: give [evaluate.gap.other] seconds, or read a series whose \
                 writer declares a rate"
                    .to_owned()
            ]
        );
        assert_eq!(
            problems(Some(10.0), None),
            vec![
                "the other spans gaps up to a cap, and the run gives none: give [evaluation] \
                 gap_multiple, or [evaluate.gap.other] multiple or seconds"
                    .to_owned()
            ]
        );
    }

    #[test]
    fn a_base_with_no_cap_is_refused_beside_the_other_naming_each() {
        let kind = TwoInputsKind::<Difference>::new();

        let problems = kind
            .build(
                &toml::Table::new(),
                &[input("car"), interpolated_input(None)],
                &[None, Some(Gap::Multiple(width(3.0)))],
            )
            .err()
            .expect("neither has a cap");

        assert_eq!(problems.len(), 2, "{problems:?}");
        assert!(
            problems[0].starts_with("the base is the base, whose gaps are found against a cap"),
            "{problems:?}"
        );
        assert!(
            problems[1].starts_with("the other spans gaps"),
            "{problems:?}"
        );
    }

    /// The other input held while no older than `cap` s, the base's gaps
    /// found past 1 s.
    fn held_within(cap: f64) -> TwoInputsEvaluation<Difference> {
        TwoInputsEvaluation::new(
            Difference {
                interpolated: false,
                drifted: false,
            },
            Cap::in_seconds(width(1.0)),
            GapRule::Hold(Some(Cap::in_seconds(width(cap)))),
        )
    }

    #[test]
    fn an_uncovered_base_value_is_counted_before_the_first_in_a_gap_or_after_the_last() {
        // The other input at 1 and 3, each held 0.5 s: the base at 0.5 is
        // before the first, at 2 in a gap, at 4 after the last.
        let mut evaluation = held_within(0.5);
        evaluation.receive(1, at(1.0), &other(10.0));
        evaluation.receive(1, at(3.0), &other(30.0));
        for t in [0.5, 1.0, 2.0, 3.0, 4.0] {
            evaluation.receive(0, at(t), &100.0);
        }

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            other_uncovered(&derived),
            UncoveredCounts {
                before_first: 1,
                in_gap: 1,
                after_last: 1,
            }
        );
        assert_eq!((derived.base_samples, derived.uncovered), (5, 3));
        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 2);
    }

    #[test]
    fn a_row_follows_a_gap_in_the_other_input_or_in_the_base_wider_than_its_cap() {
        // Rows at 1, 1.5 (the base 0.5 s on), 3 (after the other input's
        // gap at 2.5) and 4.5 (the base 1.5 s on, past its 1 s cap).
        let mut evaluation = held_within(0.5);
        for t in [1.0, 3.0, 4.5] {
            evaluation.receive(1, at(t), &other(t));
        }
        for t in [1.0, 1.5, 2.5, 3.0, 4.5] {
            evaluation.receive(0, at(t), &100.0);
        }

        let follows: Vec<(f64, bool)> = evaluation
            .line_up(at(10.0))
            .rows
            .iter()
            .map(|row| (row.at.0, row.follows_gap))
            .collect();

        assert_eq!(
            follows,
            vec![(1.0, false), (1.5, false), (3.0, true), (4.5, true)]
        );
        assert_eq!(
            derive_all(&evaluation, 10.0).coverage[0],
            Coverage::Base { gaps: 1 }
        );
    }

    #[test]
    fn a_row_follows_base_values_the_kind_couldnt_blend_the_other_at() {
        // The other input at 1, 2 and 3; the pair from 2 can't be blended,
        // so the base at 2.5 gives no row and the one at 3 follows it.
        let mut evaluation = TwoInputsEvaluation::new(
            Difference {
                interpolated: true,
                drifted: false,
            },
            Cap::in_seconds(width(10.0)),
            GapRule::Interpolate(
                |before, after, fraction| {
                    if *before == 20.0 {
                        Err(InputRejection("two values it can't blend"))
                    } else {
                        blend_numbers(before, after, fraction)
                    }
                },
                Cap::in_seconds(width(10.0)),
            ),
        );
        for t in [1.0, 2.0, 3.0] {
            evaluation.receive(1, at(t), &other(10.0 * t));
        }
        for t in [1.5, 2.5, 3.0] {
            evaluation.receive(0, at(t), &100.0);
        }

        let follows: Vec<(f64, bool)> = evaluation
            .line_up(at(10.0))
            .rows
            .iter()
            .map(|row| (row.at.0, row.follows_gap))
            .collect();

        assert_eq!(follows, vec![(1.5, false), (3.0, true)]);
    }

    /// The facts each row of `evaluation` up to 10 s reaches compute with.
    fn facts(evaluation: &TwoInputsEvaluation<Difference>) -> Vec<RowFacts> {
        as_computed(&evaluation.line_up(at(10.0)).rows)
            .iter()
            .map(|row| row.values.0)
            .collect()
    }

    #[test]
    fn compute_is_told_whether_a_row_follows_a_gap_and_whether_the_held_value_is_new() {
        // The other input at 1, 3 and 4.5, held 0.5 s: the row at 1.5 holds
        // the value the row at 1 had; the one at 3 follows the gap at 2.5.
        let mut evaluation = held_within(0.5);
        for t in [1.0, 3.0, 4.5] {
            evaluation.receive(1, at(t), &other(t));
        }
        for t in [1.0, 1.5, 2.5, 3.0, 4.5] {
            evaluation.receive(0, at(t), &100.0);
        }

        let given: Vec<(f64, bool, Option<bool>)> = facts(&evaluation)
            .iter()
            .map(|row| (row.at.0, row.follows_gap, row.other_fresh))
            .collect();

        assert_eq!(
            given,
            vec![
                (1.0, false, Some(true)),
                (1.5, false, Some(false)),
                (3.0, true, Some(true)),
                (4.5, true, Some(true)),
            ]
        );
    }

    #[test]
    fn a_replaced_held_value_is_fresh_at_its_first_row_only() {
        // The value at 2.5 replaces the one at 1 between the rows at 2 and 3;
        // the one at 3 comes at a row's own stamp.
        let mut evaluation = held();
        for t in [1.0, 2.5, 3.0] {
            evaluation.receive(1, at(t), &other(t));
        }
        for t in [1.0, 2.0, 3.0, 3.5] {
            evaluation.receive(0, at(t), &100.0);
        }

        let fresh: Vec<Option<bool>> = facts(&evaluation)
            .iter()
            .map(|row| row.other_fresh)
            .collect();

        assert_eq!(
            fresh,
            vec![Some(true), Some(false), Some(true), Some(false)]
        );
    }

    #[test]
    fn an_interpolated_value_has_no_freshness() {
        let mut evaluation = interpolated();
        evaluation.receive(1, at(1.0), &other(10.0));
        evaluation.receive(1, at(3.0), &other(30.0));
        for t in [1.0, 2.0, 3.0] {
            evaluation.receive(0, at(t), &100.0);
        }

        let fresh: Vec<Option<bool>> = facts(&evaluation)
            .iter()
            .map(|row| row.other_fresh)
            .collect();

        assert_eq!(fresh, vec![None; 3]);
    }

    #[test]
    fn a_base_value_between_two_values_the_kind_cant_blend_is_refused_not_uncovered() {
        const UNBLENDABLE: InputRejection = InputRejection("two values it can't blend");
        let mut evaluation = TwoInputsEvaluation::new(
            Difference {
                interpolated: true,
                drifted: false,
            },
            Cap::in_seconds(width(10.0)),
            GapRule::Interpolate(|_, _, _| Err(UNBLENDABLE), Cap::in_seconds(width(10.0))),
        );
        evaluation.receive(1, at(1.0), &other(10.0));
        evaluation.receive(1, at(3.0), &other(30.0));
        for t in [1.0, 2.0, 2.5, 3.0] {
            evaluation.receive(0, at(t), &100.0);
        }

        let derived = derive_all(&evaluation, 10.0);

        assert_eq!(
            (derived.uncovered, other_uncovered(&derived)),
            (0, UncoveredCounts::default())
        );
        assert_eq!(
            derived.not_computed,
            Refused {
                count: 2,
                first: Some(UNBLENDABLE),
            }
        );
        assert_eq!(series(&derived, DIFFERENCE).samples().len(), 2);
        assert_eq!(
            series(&derived, DIFFERENCE).rejection(),
            Some(Rejection::Evaluation(UNBLENDABLE))
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
        assert_eq!(
            (derived.base_samples, other_uncovered(&derived)),
            (3, UncoveredCounts::default())
        );
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
        assert_eq!(
            (derived.base_samples, other_uncovered(&derived)),
            (2, before_first(1))
        );
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
        assert_eq!(
            (derived.base_samples, other_uncovered(&derived)),
            (
                5,
                UncoveredCounts {
                    before_first: 1,
                    in_gap: 0,
                    after_last: 1,
                }
            )
        );
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
        assert_eq!(
            (derived.base_samples, other_uncovered(&derived)),
            (3, before_first(1))
        );
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
