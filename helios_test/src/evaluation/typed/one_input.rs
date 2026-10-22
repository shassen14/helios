//! [`OneInput`]: an evaluator kind of one input, the base, as its author
//! writes it. [`OneInputKind`] registers one, and each entry's evaluation
//! keeps the input's values in a [`RoleBuffer`], each one a sample.

use super::{
    buffer::{LinedUp, RoleBuffer, Row},
    component::Component,
    derive::derive,
    menu::{metrics_of, offers_its_menu},
    params,
};
use crate::evaluation::{
    config::Gap,
    derived::{Coverage, Derived, Refused},
    kind::{BoundInput, Cap, CapUse, Evaluator, EvaluatorKind, InputRejection, Metric, Role},
};

use helios_core::prelude::MonotonicTime;

use serde::de::DeserializeOwned;

use std::{any::Any, marker::PhantomData};

/// _Pre-step._ An evaluator kind of one input: its name, its parameters, the
/// input's name, payload and kept value, the metrics it offers, and what it
/// computes at each of the input's values, in stamp order. The kind is its
/// own evaluator, made by [`bind`](Self::bind) for one entry.
pub trait OneInput: Send + Sync + Sized + 'static {
    /// The name a run file's `kind` gives.
    const NAME: &'static str;

    /// The input's name in `[evaluate.inputs]`: each of its values is one
    /// sample.
    const BASE: &'static str;

    /// An entry's `[evaluate.params]`, read strictly: a struct with
    /// `#[serde(deny_unknown_fields)]`, or [`NoParams`](super::NoParams).
    type Params: DeserializeOwned;

    /// The payload type the input's series carries.
    type BasePayload: 'static;

    /// What is kept of each payload.
    type BaseValue: Send + Sync + 'static;

    /// What it fits from the whole series before any sample is computed,
    /// for a value each sample is judged against that the series itself
    /// gives: a parked GPS's mean position, its scatter about it read per
    /// sample; a line fitted through a parked estimator's position, its
    /// slope the drift rate. `()` for none.
    type Fit;

    /// What it carries from one sample to the next, starting from its
    /// default: the distance travelled so far, say. `()` for none. Kept by
    /// the evaluation between ticks, so it is as shareable as the evaluation.
    type State: Default + Send + Sync;

    /// What it computes at one sample.
    type Output;

    /// What is kept of one payload, or why it is refused.
    fn keep_base(&self, payload: &Self::BasePayload) -> Result<Self::BaseValue, InputRejection>;

    /// The metrics every evaluator of this kind offers, when they are fixed,
    /// so load checks the names a run file reads. `None` when they are known
    /// only once bound.
    fn menu() -> Option<Vec<Component<Self::Output>>>;

    /// Whether parameters that parsed are ones this kind can evaluate: a
    /// policy not built yet is refused here, naming the case that will need
    /// it.
    fn check(_params: &Self::Params) -> Result<(), String> {
        Ok(())
    }

    /// The evaluator of one entry, from its parameters and the series its
    /// input was bound to, or every reason that series can't be evaluated.
    fn bind(params: Self::Params, base: &BoundInput) -> Result<Self, Vec<String>>;

    /// The metrics it offers, each a component read off the output.
    fn components(&self) -> Vec<Component<Self::Output>>;

    /// What it fits from every value, in stamp order, or why nothing can be,
    /// which refuses every sample.
    fn fit(&self, run: &[&Self::BaseValue]) -> Result<Self::Fit, InputRejection>;

    /// The output at one sample, from its value, its facts, the state the
    /// samples before it left, and what [`fit`](Self::fit) gave, or why it
    /// can't be computed. A refused sample still updates the state.
    fn compute(
        &self,
        base: &Self::BaseValue,
        facts: BaseFacts,
        state: &mut Self::State,
        fit: &Self::Fit,
    ) -> Result<Self::Output, InputRejection>;
}

/// _Per-step._ What [`OneInput::compute`] is told of one sample besides its
/// value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BaseFacts {
    /// The value's stamp.
    pub at: MonotonicTime,
    /// Whether the value before it is further back than the input's cap.
    pub follows_gap: bool,
}

/// _Pre-step._ The one-input kind `K`, registered as any other.
pub struct OneInputKind<K>(PhantomData<fn() -> K>);

impl<K> OneInputKind<K> {
    pub fn new() -> Self {
        Self(PhantomData)
    }
}

impl<K> Default for OneInputKind<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: OneInput> EvaluatorKind for OneInputKind<K> {
    fn name(&self) -> &'static str {
        K::NAME
    }

    fn roles(&self) -> Vec<Role> {
        vec![Role::of::<K::BasePayload>(K::BASE)]
    }

    fn menu(&self) -> Option<Vec<Metric>> {
        K::menu().map(|components| metrics_of(&components))
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
        let ([base], [base_gap]) = (inputs, gaps) else {
            return Err(vec![format!("{} needs its input bound", K::NAME)]);
        };
        let kind = K::bind(params, base)?;
        let base_cap = Cap::of(K::BASE, base, *base_gap, CapUse::FindsGaps)
            .map_err(|problem| vec![problem])?;
        let evaluation = OneInputEvaluation::new(kind, base_cap);
        offers_its_menu(K::NAME, self.menu(), &evaluation.metrics())?;
        Ok(Box::new(evaluation))
    }
}

/// _Per-step._ One entry's evaluation by the one-input kind `K`: the
/// components it offers, the cap the input's gaps are found against, and
/// its values and refusals.
struct OneInputEvaluation<K: OneInput> {
    kind: K,
    components: Vec<Component<K::Output>>,
    base_cap: Cap,
    base: RoleBuffer<K::BaseValue>,
    refused: [Refused; 1],
}

impl<K: OneInput> OneInputEvaluation<K> {
    /// An evaluation by `kind`, its gaps found against `base_cap`, nothing
    /// kept yet.
    fn new(kind: K, base_cap: Cap) -> Self {
        Self {
            components: kind.components(),
            base_cap,
            base: RoleBuffer::default(),
            refused: Default::default(),
            kind,
        }
    }

    /// Each value stamped at or before `until`, each a row, which follows a
    /// gap when the value before it is further back than the cap.
    fn line_up(&self, until: MonotonicTime) -> LinedUp<(BaseFacts, &K::BaseValue)> {
        let base = self.base.up_to(until);
        let mut gaps = 0;
        let mut previous: Option<MonotonicTime> = None;
        let rows = base
            .iter()
            .map(|(at, value)| {
                let follows_gap =
                    previous.is_some_and(|previous| !self.base_cap.spans(at.0 - previous.0));
                gaps += usize::from(follows_gap);
                previous = Some(*at);
                Row {
                    at: *at,
                    follows_gap,
                    values: (
                        BaseFacts {
                            at: *at,
                            follows_gap,
                        },
                        value,
                    ),
                }
            })
            .collect();
        LinedUp {
            base_samples: base.len(),
            uncovered: 0,
            coverage: vec![Coverage::Base { gaps }],
            unblended: Refused::default(),
            rows,
        }
    }
}

impl<K: OneInput> Evaluator for OneInputEvaluation<K> {
    fn metrics(&self) -> Vec<Metric> {
        metrics_of(&self.components)
    }

    fn receive(&mut self, role: usize, at: MonotonicTime, payload: &dyn Any) {
        // Setup feeds only the one role it bound.
        if role != 0 {
            return;
        }
        let kept = self
            .base
            .keep(at, payload, |payload| self.kind.keep_base(payload));
        if let Err(rejection) = kept {
            self.refused[0].add(rejection);
        }
    }

    fn caps(&self) -> Vec<Option<Cap>> {
        vec![Some(self.base_cap)]
    }

    fn refused(&self) -> &[Refused] {
        &self.refused
    }

    fn derive(&self, until: MonotonicTime, wanted: &[usize]) -> Derived {
        let mut state = K::State::default();
        derive(
            self.line_up(until),
            |run| {
                let run: Vec<_> = run.iter().map(|(_, value)| *value).collect();
                self.kind.fit(&run)
            },
            |(facts, value), fit| self.kind.compute(value, facts, &mut state, fit),
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
        evaluation::{
            typed::{projections::number, NoParams},
            GapWidth,
        },
    };

    use helios_core::kernel::si_unit::SiUnit;
    use helios_core::prelude::AgentId;

    /// Reads finite `f64`s, kept as they are. Fits their mean, refusing a
    /// run holding a negative one, and offers each value's deviation from
    /// it and how many samples since the last gap; a value of zero is one it
    /// can't compute. Bound to instance [`DRIFTED`], it offers one metric
    /// fewer than its menu.
    struct Deviation {
        drifted: bool,
    }

    /// The instance [`Deviation`] binds to offering less than its menu.
    const DRIFTED: &str = "drifted";

    /// One sample: its deviation from the mean, and the samples since the
    /// last gap, itself included.
    #[derive(Clone, Copy)]
    struct Values {
        deviation: f64,
        run_length: f64,
    }

    impl OneInput for Deviation {
        const NAME: &'static str = "deviation";
        const BASE: &'static str = "value";
        type Params = NoParams;
        type BasePayload = f64;
        type BaseValue = f64;
        type Fit = f64;
        type State = f64;
        type Output = Values;

        fn keep_base(&self, payload: &f64) -> Result<f64, InputRejection> {
            number(payload)
        }

        fn menu() -> Option<Vec<Component<Values>>> {
            Some(components())
        }

        fn bind(_params: NoParams, base: &BoundInput) -> Result<Self, Vec<String>> {
            Ok(Self {
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

        fn fit(&self, run: &[&f64]) -> Result<f64, InputRejection> {
            if run.iter().any(|value| **value < 0.0) {
                return Err(InputRejection("a negative value"));
            }
            Ok(run.iter().copied().sum::<f64>() / run.len().max(1) as f64)
        }

        fn compute(
            &self,
            value: &f64,
            facts: BaseFacts,
            run_length: &mut f64,
            mean: &f64,
        ) -> Result<Values, InputRejection> {
            *run_length = if facts.follows_gap {
                1.0
            } else {
                *run_length + 1.0
            };
            if *value == 0.0 {
                return Err(InputRejection("a zero value"));
            }
            Ok(Values {
                deviation: value - mean,
                run_length: *run_length,
            })
        }
    }

    fn components() -> Vec<Component<Values>> {
        vec![
            Component::each("deviation", SiUnit::Meters, |v: &Values| Ok(v.deviation)),
            Component::each("run_length", SiUnit::Dimensionless, |v: &Values| {
                Ok(v.run_length)
            }),
        ]
    }

    const DEVIATION: usize = 0;
    const RUN_LENGTH: usize = 1;

    fn width(value: f64) -> GapWidth {
        GapWidth::try_from(value).expect("positive and finite")
    }

    /// Gaps found past 1 s.
    fn gap() -> Option<Gap> {
        Some(Gap::Seconds(width(1.0)))
    }

    fn at(t: f64) -> MonotonicTime {
        MonotonicTime(t)
    }

    fn input(instance: Option<&str>) -> BoundInput {
        BoundInput {
            agent: AgentId::new("car"),
            instance: instance.map(str::to_owned),
            expressed_in: None,
            convention: None,
            rate_hz: None,
            writer: None,
        }
    }

    /// An evaluation of `values`, sent in the order given, gaps found past
    /// 1 s.
    fn evaluated(values: &[(f64, f64)]) -> Box<dyn Evaluator> {
        let mut evaluation = OneInputKind::<Deviation>::new()
            .build(&toml::Table::new(), &[input(None)], &[gap()])
            .unwrap_or_else(|problems| panic!("{problems:?}"));
        for (t, value) in values {
            evaluation.receive(0, at(*t), value);
        }
        evaluation
    }

    fn values(derived: &Derived, metric: usize) -> Vec<(f64, f64)> {
        derived.metrics[metric]
            .as_ref()
            .expect("derived")
            .samples()
            .iter()
            .map(|sample| match sample.value {
                AssertionValue::Float(value) => (sample.timestamp.0, value),
                ref other => panic!("a float, not {other:?}"),
            })
            .collect()
    }

    #[test]
    fn its_input_is_one_role_and_its_menu_is_its_components() {
        let kind = OneInputKind::<Deviation>::new();
        let evaluation = evaluated(&[]);

        assert_eq!(kind.roles(), vec![Role::of::<f64>("value")]);
        assert_eq!(kind.menu(), Some(evaluation.metrics()));
    }

    #[test]
    fn each_value_is_a_sample_judged_against_the_fit_in_stamp_order() {
        let evaluation = evaluated(&[(0.2, 3.0), (0.0, 1.0), (0.1, 2.0)]);

        let derived = evaluation.derive(at(1.0), &[DEVIATION]);

        assert_eq!(
            values(&derived, DEVIATION),
            vec![(0.0, -1.0), (0.1, 0.0), (0.2, 1.0)]
        );
        assert_eq!(derived.base_samples, 3);
        assert_eq!(derived.uncovered, 0);
        assert!(derived.metrics[RUN_LENGTH].is_none(), "not wanted");
    }

    #[test]
    fn a_gap_wider_than_the_cap_is_counted_and_told_to_the_next_sample() {
        let evaluation = evaluated(&[(0.0, 1.0), (0.5, 1.0), (2.0, 1.0), (2.5, 1.0)]);

        let derived = evaluation.derive(at(3.0), &[RUN_LENGTH]);

        assert_eq!(
            values(&derived, RUN_LENGTH),
            vec![(0.0, 1.0), (0.5, 2.0), (2.0, 1.0), (2.5, 2.0)]
        );
        assert_eq!(derived.coverage, vec![Coverage::Base { gaps: 1 }]);
    }

    #[test]
    fn values_past_until_are_left_for_later() {
        let evaluation = evaluated(&[(0.0, 1.0), (0.5, 3.0), (2.0, 5.0)]);

        let derived = evaluation.derive(at(1.0), &[DEVIATION]);

        assert_eq!(values(&derived, DEVIATION), vec![(0.0, -1.0), (0.5, 1.0)]);
    }

    #[test]
    fn a_refused_payload_and_a_refused_sample_are_counted_apart() {
        let evaluation = evaluated(&[(0.0, 2.0), (0.1, f64::NAN), (0.2, 0.0)]);

        let derived = evaluation.derive(at(1.0), &[DEVIATION]);

        assert_eq!(evaluation.refused()[0].count, 1);
        assert_eq!(
            derived.base_samples, 2,
            "the refused payload was never kept"
        );
        assert_eq!(derived.not_computed.count, 1);
        assert_eq!(values(&derived, DEVIATION), vec![(0.0, 1.0)]);
    }

    #[test]
    fn a_refused_fit_refuses_every_sample() {
        let evaluation = evaluated(&[(0.0, 2.0), (0.1, -1.0)]);

        let derived = evaluation.derive(at(1.0), &[DEVIATION]);

        assert_eq!(derived.not_computed.count, 2);
        assert!(values(&derived, DEVIATION).is_empty());
    }

    #[test]
    fn build_refuses_a_missing_cap_a_missing_input_and_a_drifted_menu() {
        let kind = OneInputKind::<Deviation>::new();
        let table = toml::Table::new();

        let no_cap = kind.build(&table, &[input(None)], &[None]).err();
        let no_input = kind.build(&table, &[], &[]).err();
        let drifted = kind.build(&table, &[input(Some(DRIFTED))], &[gap()]).err();

        assert!(no_cap.is_some_and(|problems| problems[0].contains("gap_multiple")));
        assert_eq!(
            no_input,
            Some(vec!["deviation needs its input bound".to_owned()])
        );
        assert!(drifted.is_some_and(|problems| problems[0].contains("offers exactly its menu")));
    }
}
