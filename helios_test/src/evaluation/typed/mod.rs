//! Typed evaluator kinds: a kind written with its own types, its roles a
//! [`Base`] and up to two [`Input`]s, its output read by [`Component`]s, and
//! erased once by [`Typed`] into the form the collector feeds.

mod adapter;
mod buffer;
mod component;
mod evaluation;
mod input;
mod kind;
mod params;
pub mod projections;
mod two_inputs;

pub use adapter::Typed;
pub use buffer::Lining;
pub use component::Component;
pub use input::{Base, Input, Inputs};
pub use kind::{OutputOf, RowOf, TypedEvaluator, TypedKind};
pub use params::NoParams;
pub use two_inputs::{TwoInputs, TwoInputsKind};
