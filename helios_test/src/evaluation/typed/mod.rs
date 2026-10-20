//! Typed evaluator kinds: a kind written with its own types, its roles a
//! [`Base`] and up to two [`Input`]s, its output read by [`Component`]s, and
//! erased once by [`Typed`] into the form the collector feeds.

mod adapter;
mod component;
mod evaluation;
mod input;
mod kind;
pub mod projections;

pub use adapter::Typed;
pub use component::Component;
pub use input::{Base, Input, Inputs};
pub use kind::{NoParams, OutputOf, RowOf, TypedEvaluator, TypedKind};
