use helios_core::prelude::AgentId;
use helios_runtime::observe::path::{AgentPathParts, PathPart, PathSubject, SUBJECT_WORDS};

use serde::Deserialize;
use std::{fmt, sync::Arc};

/// _Shared (pre + per)._ The watcher path an assertion watches, held whole
/// and in its parts: `agent.<agent>.node.<node>.<leaf>` or
/// `agent.<agent>.channel.<channel>`.
///
/// Only a path that splits into its parts becomes a target, so a run file
/// naming anything else is refused when it is read, before a sim starts.
/// Whether the agent reports what the path names is checked later, against
/// the built pipelines.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Deserialize)]
#[serde(try_from = "String")]
pub struct AssertionTarget {
    path: String,
    agent: AgentId,
    subject: TargetSubject,
}

impl AssertionTarget {
    /// The target `path` names, or why it names none.
    pub fn parse(path: impl Into<String>) -> Result<Self, MalformedTarget> {
        let path = path.into();
        let parts = match AgentPathParts::split(&path) {
            Ok(parts) => parts,
            Err(part) => return Err(MalformedTarget { path, part }),
        };
        let agent = AgentId::new(parts.agent);
        let subject = TargetSubject::from(parts.subject);

        Ok(Self {
            path,
            agent,
            subject,
        })
    }

    pub fn as_str(&self) -> &str {
        &self.path
    }

    pub fn agent(&self) -> &AgentId {
        &self.agent
    }

    pub fn subject(&self) -> &TargetSubject {
        &self.subject
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

/// _Shared (pre + per)._ What kind of value a target names, with its parts
/// after the subject word.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum TargetSubject {
    /// What a pipeline node said about itself, or wrote to an output channel.
    Node {
        node: Arc<str>,
        /// The leaf or output channel segment, dots included.
        leaf: Arc<str>,
    },
    /// A value on one of the agent's channels, by its segment.
    Channel { channel: Arc<str> },
}

impl From<PathSubject<'_>> for TargetSubject {
    fn from(subject: PathSubject<'_>) -> Self {
        match subject {
            PathSubject::Node { node, leaf } => TargetSubject::Node {
                node: node.into(),
                leaf: leaf.into(),
            },
            PathSubject::Channel { channel } => TargetSubject::Channel {
                channel: channel.into(),
            },
        }
    }
}

/// _Pre-step._ A run-file target that doesn't split into the parts of a
/// watcher path, and the first part that is wrong.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error(
    "target '{path}' is not a watcher path agent.<agent>.node.<node>.<leaf> \
     or agent.<agent>.channel.<channel>: {}",
    what_is_wrong(*part)
)]
pub struct MalformedTarget {
    path: String,
    part: PathPart,
}

/// What is wrong with a target whose `part` is the first that is.
fn what_is_wrong(part: PathPart) -> String {
    match part {
        PathPart::Root => "it starts with something else".to_owned(),
        PathPart::Agent => "the agent is missing".to_owned(),
        PathPart::Subject => format!(
            "the word after the agent is missing or not one of {}",
            SUBJECT_WORDS.join(", ")
        ),
        PathPart::Node => "the node is missing".to_owned(),
        PathPart::Leaf => "the leaf is missing".to_owned(),
        PathPart::Channel => "the channel is missing".to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_keeps_the_path_and_its_parts() {
        let target = AssertionTarget::parse("agent.car.node.estimator.aiding.gps.nis")
            .expect("valid target");

        assert_eq!(target.as_str(), "agent.car.node.estimator.aiding.gps.nis");
        assert_eq!(target.agent(), &AgentId::new("car"));
        assert_eq!(
            target.subject(),
            &TargetSubject::Node {
                node: "estimator".into(),
                leaf: "aiding.gps.nis".into(),
            }
        );
    }

    #[test]
    fn parse_keeps_a_channel_target() {
        let target = AssertionTarget::parse("agent.car.channel.truth.tf").expect("valid target");

        assert_eq!(
            target.subject(),
            &TargetSubject::Channel {
                channel: "truth.tf".into(),
            }
        );
    }

    #[test]
    fn parse_refuses_a_path_with_no_leaf() {
        assert_eq!(
            AssertionTarget::parse("agent.car.node.estimator"),
            Err(MalformedTarget {
                path: "agent.car.node.estimator".to_owned(),
                part: PathPart::Leaf,
            })
        );
    }

    #[test]
    fn a_malformed_target_says_which_part_is_wrong() {
        let error = AssertionTarget::parse("agent.car.node").expect_err("no node or leaf");

        assert!(
            error.to_string().ends_with("the node is missing"),
            "{error}"
        );
    }

    #[test]
    fn a_target_without_a_subject_says_which_subjects_exist() {
        let error =
            AssertionTarget::parse("agent.car.estimator.speed").expect_err("no subject word");

        assert!(
            error.to_string().ends_with("not one of node, channel"),
            "{error}"
        );
    }

    #[test]
    fn parse_refuses_a_path_outside_the_agent_root() {
        assert!(AssertionTarget::parse("car.estimator.speed.x").is_err());
    }
}
