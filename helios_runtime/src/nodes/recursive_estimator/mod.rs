//! Recursive-estimator family: one node kind for the predict → update →
//! publish loop, assembled from the estimator components (`nodes::estimation`)
//! its section names.
//!
//! - `node` — `RecursiveEstimatorNode`, the loop, and the skip warnings it
//!   shares with the legacy Gaussian-estimator node.
//! - `config` — `RecursiveEstimatorConfig`, the `[nodes.<name>]` section.
//! - `register` — registers the `RecursiveEstimator` kind and its factory.

mod config;
mod node;
mod register;

pub(crate) use node::{
    aiding_drop_cause, passes_warn_throttle, predict_skip_cause, publish_estimate,
};
pub(crate) use register::register;
