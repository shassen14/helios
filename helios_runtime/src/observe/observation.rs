//! A value a node emitted for watchers: tests, visualizers, recorders.
//!
//! Observations leave the brain and are never read back by it; the bus
//! carries what nodes consume. A node emits raw samples, and whoever watches
//! computes statistics over them (a mean, a settling time, a rate across
//! runs), so the node never fixes which summaries exist.

use crate::observe::path::agent_path;

use helios_core::prelude::{AgentId, MonotonicTime};

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

/// An observation with the agent whose pipeline emitted it: what a host hands
/// its sinks.
///
/// The agent is a field rather than part of a path string, so tagging a
/// sample builds no string; every field is shared, so a sink keeps a sample
/// by cloning it cheaply.
#[derive(Debug, Clone, PartialEq)]
pub struct AgentObservation {
    pub agent: AgentId,
    pub observation: Observation,
}

impl AgentObservation {
    /// The sample's full watcher path, `agent.<agent>.<node>.<leaf>`: the
    /// spelling sinks request it by.
    pub fn path(&self) -> String {
        agent_path(
            self.agent.as_str(),
            &self.observation.node,
            &self.observation.leaf,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_names_the_agent_node_and_leaf() {
        let tagged = AgentObservation {
            agent: AgentId::new("car"),
            observation: Observation {
                node: "estimator".into(),
                leaf: "aiding.gps.nis".into(),
                timestamp: MonotonicTime(1.0),
                value: ObservedValue::Scalar(0.5),
            },
        };

        assert_eq!(tagged.path(), "agent.car.estimator.aiding.gps.nis");
    }
}
