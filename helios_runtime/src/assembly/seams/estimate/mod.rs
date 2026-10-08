//! The estimate seam: turns the `[estimate]` section into the relay that
//! forwards one estimator's state as the agent's estimate.
//!
//! Each estimator writes its state on a channel of its own. The seam finds the
//! source's state output among the built nodes and adds one
//! [`EstimateRelay`] from it onto the fixed [`estimate`] channel. The relay
//! alone publishes the `odom → base_link` edge, so the estimators the section
//! does not name run in shadow.
//!
//! - `config` — the section's config.
//! - `pass` — the pass that builds the seam's node.
//!
//! [`EstimateRelay`]: crate::nodes::estimate_relay::EstimateRelay
//! [`estimate`]: crate::channels::estimate::estimate

mod config;
mod pass;

pub use self::config::EstimateSeamConfig;
pub(in crate::assembly) use self::pass::estimate_relay;
