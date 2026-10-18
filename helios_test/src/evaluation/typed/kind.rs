//! [`TypedKind`] and [`TypedEvaluator`]: an evaluator kind as its author
//! writes it, every value its own type. [`Typed`](super::Typed) erases one
//! into the form the machinery stores.

use super::{component::Component, input::Inputs};
use crate::evaluation::kind::{BoundInput, InputRejection};

use serde::{de::DeserializeOwned, Deserialize};

/// _Pre-step._ An evaluator kind, typed: its name, its parameters, its
/// inputs, the metrics it offers, and how one entry's evaluator is bound.
pub trait TypedKind: Send + Sync + 'static {
    /// The name a run file's `kind` gives.
    const NAME: &'static str;

    /// An entry's `[evaluate.params]`, read strictly: a struct with
    /// `#[serde(deny_unknown_fields)]`, or [`NoParams`].
    type Params: DeserializeOwned;

    /// One entry's evaluator.
    type Evaluator: TypedEvaluator;

    /// Its inputs, one to three, in role order, each with what it keeps of
    /// a value and how it lines up in time; exactly one is the base.
    fn inputs(&self) -> <Self::Evaluator as TypedEvaluator>::Inputs;

    /// The metrics every evaluator of this kind offers, when they are fixed,
    /// so load checks the names a run file reads. `None` when they are known
    /// only once bound, such as one per block of an estimator's state.
    fn menu(&self) -> Option<Vec<Component<OutputOf<Self>>>>;

    /// Whether parameters that parsed are ones this kind can evaluate: a
    /// policy not built yet is refused here, naming the case that will need
    /// it.
    fn check(&self, _params: &Self::Params) -> Result<(), String> {
        Ok(())
    }

    /// The evaluator of one entry, from its parameters and the series each
    /// role was bound to, in role order, or every reason those series can't
    /// be compared.
    fn bind(
        &self,
        params: Self::Params,
        inputs: &[BoundInput],
    ) -> Result<Self::Evaluator, Vec<String>>;
}

/// _Per-step._ One entry's evaluation, typed: what it fits over the run,
/// and what it computes at each sample, which its components read.
pub trait TypedEvaluator: Send + Sync + Sized + 'static {
    /// Its inputs, as [`TypedKind::inputs`] declares them.
    type Inputs: Inputs<Self>;

    /// What it fits over the whole run before any sample is computed: how
    /// one tree's root sits in another's, say. `()` for none.
    type Fit;

    /// What it computes at one sample: for a comparison, the subject minus
    /// the reference, the subject being what is graded and the reference
    /// what it is judged against; for a kind of one input, what it reads of
    /// that input.
    type Output;

    /// The metrics it offers, each a component read off the output.
    fn components(&self) -> Vec<Component<Self::Output>>;

    /// What it fits from every covered sample, in stamp order, or why
    /// nothing can be, which refuses every sample.
    fn fit(&self, run: &[RowOf<'_, Self>]) -> Result<Self::Fit, InputRejection>;

    /// The output at one sample, from every role's value lined up there and
    /// what [`fit`](Self::fit) gave, or why the two can't be compared.
    fn compute(
        &self,
        row: RowOf<'_, Self>,
        fit: &Self::Fit,
    ) -> Result<Self::Output, InputRejection>;
}

/// One sample's values, one reference per role, in role order.
pub type RowOf<'a, E> = <<E as TypedEvaluator>::Inputs as Inputs<E>>::Row<'a>;

/// The output type of a kind's evaluator.
pub type OutputOf<K> = <<K as TypedKind>::Evaluator as TypedEvaluator>::Output;

/// _Pre-step._ The parameters of a kind that reads none: any key is
/// refused.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NoParams {}
