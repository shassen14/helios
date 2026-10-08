//! Recursive-estimator family: one node kind for the predict → update →
//! publish loop, assembled from the estimator components (`nodes::estimation`)
//! its section names.
//!
//! - `node` — `RecursiveEstimatorNode`, the loop, and its skip warnings.
//! - `config` — `RecursiveEstimatorConfig`, the `[nodes.<name>]` section.
//! - `nis_health` — `NisWindow`, the windowed-NIS health check on one aiding
//!   sensor.
//! - `register` — registers the `RecursiveEstimator` kind and its factory.

mod config;
mod nis_health;
mod node;
mod register;

pub(crate) use register::register;
