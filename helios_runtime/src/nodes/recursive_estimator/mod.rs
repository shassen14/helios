//! Recursive-estimator family: one node kind for the predict → update →
//! publish loop, assembled from the estimator components (`nodes::estimation`)
//! its section names.
//!
//! - `node` — `RecursiveEstimatorNode`, the loop, and its predict-skip
//!   warnings.
//! - `aiding` — `Aiding`, one aiding source: its updates, NIS window, reports
//!   and drop warnings.
//! - `throttle` — the rate limit both kinds of skip warning share.
//! - `config` — `RecursiveEstimatorConfig`, the `[nodes.<name>]` section.
//! - `nis_health` — `NisWindow`, the windowed-NIS health check on one aiding
//!   sensor.
//! - `leaves` — `AidingLeaf`, what each aiding source reports to watchers.
//! - `register` — registers the `RecursiveEstimator` kind and its factory.

mod aiding;
mod config;
mod leaves;
mod nis_health;
mod node;
mod register;
#[cfg(test)]
mod test_fixtures;
mod throttle;

pub(crate) use register::register;
