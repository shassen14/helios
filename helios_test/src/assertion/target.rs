use helios_core::prelude::AgentId;
use helios_runtime::observe::path::{AgentPathParts, PathPart};

use serde::Deserialize;
use std::{fmt, sync::Arc};

/// _Shared (pre + per)._ The watcher path an assertion watches,
/// `agent.<agent>.<node>.<leaf>`, held whole and in its parts.
///
/// Only a path that splits into an agent, a node and a leaf becomes a
/// target, so a run file naming anything else is refused when it is read,
/// before a sim starts.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Deserialize)]
#[serde(try_from = "String")]
pub struct AssertionTarget {
    path: String,
    agent: AgentId,
    node: Arc<str>,
    leaf: Arc<str>,
}

impl AssertionTarget {
    /// The target `path` names, or why it names none.
    pub fn parse(path: impl Into<String>) -> Result<Self, MalformedTarget> {
        let path = path.into();
        let parts = match AgentPathParts::split(&path) {
            Ok(parts) => parts,
            Err(part) => return Err(MalformedTarget { path, part }),
        };
        let (agent, node, leaf) = (
            AgentId::new(parts.agent),
            parts.node.into(),
            parts.rest.into(),
        );

        Ok(Self {
            path,
            agent,
            node,
            leaf,
        })
    }

    pub fn as_str(&self) -> &str {
        &self.path
    }

    pub fn agent(&self) -> &AgentId {
        &self.agent
    }

    pub fn node(&self) -> &Arc<str> {
        &self.node
    }

    /// The leaf or channel segment, dots included.
    pub fn leaf(&self) -> &Arc<str> {
        &self.leaf
    }
}

impl TryFrom<String> for AssertionTarget {
    type Error = MalformedTarget;

    fn try_from(path: String) -> Result<Self, Self::Error> {
        Self::parse(path)
    }
}

impl fmt::Display for AssertionTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.path)
    }
}

/// _Pre-step._ A run-file target that doesn't split into an agent, a node
/// and a leaf, and the first part that is wrong.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error(
    "target '{path}' is not a watcher path agent.<agent>.<node>.<leaf>: {}",
    what_is_wrong(*.part)
)]
pub struct MalformedTarget {
    path: String,
    part: PathPart,
}

/// What is wrong with a target whose `part` is the first to fail.
fn what_is_wrong(part: PathPart) -> &'static str {
    match part {
        PathPart::Root => "it starts with something else",
        PathPart::Agent => "the agent is missing",
        PathPart::Node => "the node is missing",
        PathPart::Rest => "the leaf is missing",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_keeps_the_path_and_its_parts() {
        let target =
            AssertionTarget::parse("agent.car.estimator.aiding.gps.nis").expect("valid target");

        assert_eq!(target.as_str(), "agent.car.estimator.aiding.gps.nis");
        assert_eq!(target.agent(), &AgentId::new("car"));
        assert_eq!(target.node().as_ref(), "estimator");
        assert_eq!(target.leaf().as_ref(), "aiding.gps.nis");
    }

    #[test]
    fn parse_refuses_a_path_with_no_leaf() {
        assert_eq!(
            AssertionTarget::parse("agent.car.estimator"),
            Err(MalformedTarget {
                path: "agent.car.estimator".to_owned(),
                part: PathPart::Rest,
            })
        );
    }

    #[test]
    fn a_malformed_target_says_which_part_is_wrong() {
        let error = AssertionTarget::parse("agent.car").expect_err("no node or leaf");

        assert!(
            error.to_string().ends_with("the node is missing"),
            "{error}"
        );
    }

    #[test]
    fn parse_refuses_a_path_outside_the_agent_root() {
        assert!(AssertionTarget::parse("car.estimator.speed.x").is_err());
    }
}
