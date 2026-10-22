//! Typed evaluator kinds: a kind written with its own types, one trait per
//! number of inputs ([`OneInput`], [`TwoInputs`]), its output read by
//! [`Component`]s, and erased once by its shape's registry adapter
//! ([`OneInputKind`], [`TwoInputsKind`]) into the form the collector feeds.

mod buffer;
mod component;
mod derive;
mod menu;
mod one_input;
mod params;
pub mod projections;
mod two_inputs;

pub use buffer::Lining;
pub use component::Component;
pub use one_input::{BaseFacts, OneInput, OneInputKind};
pub use params::NoParams;
pub use two_inputs::{RowFacts, TwoInputs, TwoInputsKind};
