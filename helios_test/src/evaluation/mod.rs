//! Evaluations: series a run derives from recorded ones, by an evaluator kind
//! from the registry, and judges at finalize like any other series.
//!
//! An `[[evaluate]]` entry names a kind and the recorded series each of its
//! roles reads. Each value is kept as a small value of the kind's own type as
//! it arrives and the payload dropped; at finalize the roles are lined up in
//! time as the kind declares, and each metric something reads becomes a series spelled
//! `eval.inline.<name>.<metric>`, which assertions and measures name.

pub mod config;
pub mod derived;
pub mod kind;
pub mod kinds;
pub mod path;
pub mod registry;
pub mod typed;

pub use config::Evaluation;
pub use registry::{standard_evaluators, EvaluatorRegistry};
