//! Typed evaluator kinds: a kind written with its own types, its roles a
//! tuple of [`Input`]s, its output read by [`Component`]s, and erased once by
//! [`Typed`] into the form the machinery stores.

mod adapter;
mod component;
mod input;
mod kind;
pub mod projections;

pub use adapter::Typed;
#[cfg(test)]
pub(crate) use adapter::TypedEvaluation;
pub use component::Component;
pub use input::{Input, Inputs};
pub use kind::{NoParams, OutputOf, RowOf, TypedEvaluator, TypedKind};
