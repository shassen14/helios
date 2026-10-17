//! Evaluations: series a run derives from recorded ones, by an evaluator kind
//! from the registry, and judges at finalize like any other series.
//!
//! An `[[evaluate]]` entry names a kind and the recorded series each of its
//! roles reads. Each value is projected into a small value as it arrives and
//! the payload dropped; at finalize the roles are lined up in time as the
//! kind declares, and each metric becomes a series spelled
//! `eval.inline.<name>.<metric>`, which assertions and measures name.

pub mod config;
pub mod derived;
pub mod kind;
pub mod kinds;
pub mod path;
pub mod registry;

pub use config::{Align, Evaluation};
pub use registry::{standard_evaluators, EvaluatorRegistry};
