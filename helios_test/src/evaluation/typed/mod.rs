//! Typed evaluator kinds: a kind written with its own types, one trait per
//! number of inputs ([`TwoInputs`]), its output read by [`Component`]s, and
//! erased once by its shape's registry adapter ([`TwoInputsKind`]) into the
//! form the collector feeds.

mod buffer;
mod component;
mod derive;
mod params;
pub mod projections;
mod two_inputs;

pub use buffer::Lining;
pub use component::Component;
pub use params::NoParams;
pub use two_inputs::{TwoInputs, TwoInputsKind};
