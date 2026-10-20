//! [`Typed`]: a [`TypedKind`] erased into the [`EvaluatorKind`] the registry
//! holds. Parameters are read here, once for every typed kind, and each
//! entry's evaluation is a [`TypedEvaluation`].

use super::{component::Component, evaluation::TypedEvaluation, input::Inputs, kind::TypedKind};
use crate::evaluation::kind::{BoundInput, Evaluator, EvaluatorKind, Metric, Role};

use serde::Deserialize;

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
