//! [`Typed`]: a [`TypedKind`] erased into the [`EvaluatorKind`] the registry
//! holds, and [`TypedEvaluation`], its evaluator erased. Parameters are read
//! here, payloads and kept values downcast, and the run fitted, computed and
//! read into components, once for every typed kind.

use super::{
    component::Component,
    input::Inputs,
    kind::{RowOf, TypedEvaluator, TypedKind},
};
use crate::evaluation::{
    derived::Refused,
    kind::{
        BoundInput, Evaluator, EvaluatorKind, Graded, InputRejection, Metric, Projection, Role, Row,
    },
};

use helios_core::prelude::MonotonicTime;

use serde::Deserialize;
use std::any::Any;

/// A sample whose values aren't ones the evaluation kept. Its projections
/// are its own, so this is never reached.
const NOT_KEPT: InputRejection = InputRejection("a sample the evaluation didn't keep");

/// _Pre-step._ The typed kind `K`, registered as any other.
pub struct Typed<K>(pub K);

impl<K: TypedKind> Typed<K> {
    /// `params` read as `K`'s, and checked by it.
    fn read(&self, params: &toml::Table) -> Result<K::Params, String> {
        let params =
            K::Params::deserialize(params.clone()).map_err(|error| error.message().to_owned())?;
        self.0.check(&params)?;
        Ok(params)
    }
}

impl<K: TypedKind> EvaluatorKind for Typed<K> {
    fn name(&self) -> &'static str {
        K::NAME
    }

    fn roles(&self) -> Vec<Role> {
        self.0.inputs().roles()
    }

    fn menu(&self) -> Option<Vec<Metric>> {
        self.0
            .menu()
            .map(|components| components.iter().map(Component::metric).collect())
    }

    fn check_params(&self, params: &toml::Table) -> Result<(), String> {
        self.read(params)
            .map(|_| ())
            .map_err(|problem| format!("{}: {problem}", K::NAME))
    }

    fn build(
        &self,
        params: &toml::Table,
        inputs: &[BoundInput],
    ) -> Result<Box<dyn Evaluator>, Vec<String>> {
        let params = self.read(params).map_err(|problem| vec![problem])?;
        let evaluator = self.0.bind(params, inputs)?;
        Ok(Box::new(TypedEvaluation::new(evaluator, self.0.inputs())))
    }
}

/// _Per-step._ The typed evaluator `E` erased, with its inputs and the
/// components it offers.
pub(crate) struct TypedEvaluation<E: TypedEvaluator> {
    evaluator: E,
    inputs: E::Inputs,
    components: Vec<Component<E::Output>>,
}

impl<E: TypedEvaluator> TypedEvaluation<E> {
    pub(crate) fn new(evaluator: E, inputs: E::Inputs) -> Self {
        Self {
            components: evaluator.components(),
            evaluator,
            inputs,
        }
    }
}

impl<E: TypedEvaluator> Evaluator for TypedEvaluation<E> {
    fn metrics(&self) -> Vec<Metric> {
        self.components.iter().map(Component::metric).collect()
    }

    fn project(&self, role: usize, payload: &dyn Any) -> Result<Projection, InputRejection> {
        self.inputs.project(&self.evaluator, role, payload)
    }

    fn interpolate(
        &self,
        role: usize,
        before: &Projection,
        after: &Projection,
        fraction: f64,
    ) -> Option<Projection> {
        self.inputs.blend(role, before, after, fraction)
    }

    fn grade(&self, run: &[Row<'_>], wanted: &[usize], until: MonotonicTime) -> Graded {
        let mut refused = Refused::default();
        // Every wanted metric with no values, for a run refused whole: each
        // still carries the refusal to the entries reading it.
        let empty = || {
            (0..self.components.len())
                .map(|index| wanted.contains(&index).then(Vec::new))
                .collect()
        };
        let Some(rows) = run
            .iter()
            .map(|row| Some((row.at, self.inputs.row(&row.aligned)?)))
            .collect::<Option<Vec<(MonotonicTime, RowOf<'_, E>)>>>()
        else {
            run.iter().for_each(|_| refused.add(NOT_KEPT));
            return Graded {
                metrics: empty(),
                refused,
            };
        };

        let values: Vec<RowOf<'_, E>> = rows.iter().map(|(_, row)| *row).collect();
        let outputs: Vec<(MonotonicTime, E::Output)> = match self.evaluator.fit(&values) {
            Ok(fit) => rows
                .iter()
                .filter_map(|(at, row)| match self.evaluator.compute(*row, &fit) {
                    Ok(output) => Some((*at, output)),
                    Err(rejection) => {
                        refused.add(rejection);
                        None
                    }
                })
                .collect(),
            Err(rejection) => {
                rows.iter().for_each(|_| refused.add(rejection));
                return Graded {
                    metrics: empty(),
                    refused,
                };
            }
        };

        let metrics = self
            .components
            .iter()
            .enumerate()
            .map(|(index, component)| {
                wanted
                    .contains(&index)
                    .then(|| component.values(&outputs, until, &mut refused))
            })
            .collect();
        Graded { metrics, refused }
    }
}
