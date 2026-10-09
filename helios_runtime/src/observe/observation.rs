//! A value a node emitted for watchers: tests, visualizers, recorders.
//!
//! Observations leave the brain and are never read back by it; the bus
//! carries what nodes consume. A node emits raw samples, and whoever watches
//! computes statistics over them (a mean, a settling time, a rate across
//! runs), so the node never fixes which summaries exist.

use helios_core::prelude::MonotonicTime;

use std::sync::Arc;

/// One sample a node emitted, named relative to that node.
///
/// `node` + `leaf` name the value inside one pipeline; the host adds the
/// agent when it drains, since a pipeline doesn't know which agent runs it.
/// Fields are public so sinks can read them and host-side scorers can build
/// their own observations.
#[derive(Debug, Clone, PartialEq)]
pub struct Observation {
    /// The emitting node's name.
    pub node: Arc<str>,
    /// The name the node declared for this value, e.g. `aiding.gps.nis`.
    pub leaf: Arc<str>,
    /// The time the value describes, not when it was emitted: a measurement
    /// update stamps the reading's time.
    pub timestamp: MonotonicTime,
    pub value: ObservedValue,
}

/// The kinds of value an observation can carry.
///
/// Non-exhaustive: richer kinds (shared typed values, structured events) are
/// added as variants, so a `match` outside this crate needs a wildcard arm,
/// and a sink skips kinds it doesn't handle. Deliberately not `Copy`, since
/// later kinds hold shared data.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum ObservedValue {
    /// A number in SI units: a statistic such as NIS, a count (each event
    /// emits `1.0`), or a duration in seconds.
    Scalar(f64),
}

impl From<f64> for ObservedValue {
    fn from(value: f64) -> Self {
        ObservedValue::Scalar(value)
    }
}
