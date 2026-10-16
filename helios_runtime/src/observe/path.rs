//! How names are spelled inside a watcher path.
//!
//! Watchers address everything recorded about an agent by a dotted path,
//! `agent.<agent>.<subject>.…`. The subject word says what kind of value the
//! path names, and decides what follows it:
//!
//! - `node.<node>.<leaf>`: what a pipeline node said about itself, under a
//!   leaf it declared, or a value it wrote to one of its output channels.
//! - `channel.<channel>`: a value on one of the agent's channels, whoever
//!   wrote it.
//!
//! Who wrote a value is not part of its path; the path names only what the
//! value is, so swapping the writer (one estimator for another, a host for a
//! replay) keeps every path. The subject words are fixed and sit in a part no
//! config fills, so a name a profile chooses never collides with one. The
//! agent and the node each fill exactly one part, so their names must be one
//! [level](helios_core::kernel::identifier::is_level); a leaf and a channel
//! name may hold several. A channel sits in a path under its own name,
//! verbatim. A node's leaves and its output channels' names share the part
//! after the node, so the build compares the two.

use crate::port::ChannelKey;

use helios_core::kernel::identifier::LEVEL_SEPARATOR;
use helios_core::spatial::primitives::AGENT_SCOPE;

/// The subject word for what a pipeline node said or wrote.
const NODE_SUBJECT: &str = "node";

/// The subject word for a value on one of the agent's channels.
const CHANNEL_SUBJECT: &str = "channel";

/// Every subject word, in the order [`PathSubject`] declares them, for
/// messages that list what a path may hold.
pub const SUBJECT_WORDS: [&str; 2] = [NODE_SUBJECT, CHANNEL_SUBJECT];

/// A watcher path split into its parts, borrowed from the path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AgentPathParts<'a> {
    pub agent: &'a str,
    pub subject: PathSubject<'a>,
}

impl<'a> AgentPathParts<'a> {
    /// Splits `path`, the inverse of [`join`](Self::join). The agent, the
    /// subject word and a node hold no separator, so the separators after
    /// each end them, and everything after is the leaf or the channel. On
    /// failure, names the first part that is wrong: a root other than
    /// [`AGENT_SCOPE`], a subject word that isn't one of [`SUBJECT_WORDS`],
    /// or a part that is missing or empty.
    pub fn split(path: &'a str) -> Result<Self, PathPart> {
        let mut parts = path.splitn(4, LEVEL_SEPARATOR);
        if parts.next() != Some(AGENT_SCOPE) {
            return Err(PathPart::Root);
        }
        let agent = non_empty(parts.next()).ok_or(PathPart::Agent)?;
        let word = parts.next().ok_or(PathPart::Subject)?;
        let after = parts.next();
        let subject = match word {
            NODE_SUBJECT => {
                let mut node_parts = after.unwrap_or_default().splitn(2, LEVEL_SEPARATOR);
                let node = non_empty(node_parts.next()).ok_or(PathPart::Node)?;
                let leaf = non_empty(node_parts.next()).ok_or(PathPart::Leaf)?;
                PathSubject::Node { node, leaf }
            }
            CHANNEL_SUBJECT => PathSubject::Channel {
                channel: non_empty(after).ok_or(PathPart::Channel)?,
            },
            _ => return Err(PathPart::Subject),
        };

        Ok(Self { agent, subject })
    }

    /// The watcher path these parts spell. The agent and a node must each be
    /// one [level](helios_core::kernel::identifier::is_level); the build
    /// refuses names that aren't. A leaf and a channel may hold dots: a
    /// leaf a node declared (`aiding.gps.nis`) or a channel's name
    /// (`oracle.pose`).
    pub fn join(&self) -> String {
        let mut parts = vec![AGENT_SCOPE, self.agent, self.subject.word()];
        match self.subject {
            PathSubject::Node { node, leaf } => parts.extend([node, leaf]),
            PathSubject::Channel { channel } => parts.push(channel),
        }
        parts.join(LEVEL_SEPARATOR)
    }
}

/// What kind of value a watcher path names, with the parts its path holds
/// after the subject word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathSubject<'a> {
    /// What a pipeline node said about itself, or wrote to an output channel.
    Node {
        node: &'a str,
        /// The leaf or output channel name, dots included.
        leaf: &'a str,
    },
    /// A value on one of the agent's channels, by the channel's name.
    Channel { channel: &'a str },
}

impl PathSubject<'_> {
    /// The word that spells this subject in a path.
    pub fn word(&self) -> &'static str {
        match self {
            PathSubject::Node { .. } => NODE_SUBJECT,
            PathSubject::Channel { .. } => CHANNEL_SUBJECT,
        }
    }
}

/// One of the parts of a watcher path, `agent.<agent>.<subject>.…`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathPart {
    /// The leading `agent`.
    Root,
    Agent,
    /// The subject word, one of [`SUBJECT_WORDS`].
    Subject,
    /// The node, under the `node` subject.
    Node,
    /// The node's leaf or output channel name.
    Leaf,
    /// The channel's name, under the `channel` subject.
    Channel,
}

/// The watcher path of what `node` of `agent` said or wrote,
/// `agent.<agent>.node.<node>.<leaf>`. [`AgentPathParts::split`] takes it
/// apart again.
pub fn node_path(agent: &str, node: &str, leaf: &str) -> String {
    AgentPathParts {
        agent,
        subject: PathSubject::Node { node, leaf },
    }
    .join()
}

/// The watcher path of the value on `channel` of `agent`,
/// `agent.<agent>.channel.<channel>`. [`AgentPathParts::split`] takes it
/// apart again.
pub fn channel_path(agent: &str, channel: &ChannelKey) -> String {
    AgentPathParts {
        agent,
        subject: PathSubject::Channel {
            channel: channel.name(),
        },
    }
    .join()
}

/// `part` unless it is missing or empty.
fn non_empty(part: Option<&str>) -> Option<&str> {
    part.filter(|part| !part.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    fn channel_named<T: 'static>(name: &'static str) -> ChannelKey {
        ChannelKey::internal::<T>(name)
    }

    #[test]
    fn node_path_puts_the_parts_in_order_under_the_root() {
        assert_eq!(
            node_path("car", "estimator", "position"),
            "agent.car.node.estimator.position"
        );
    }

    #[test]
    fn node_path_keeps_a_dotted_rest_whole() {
        assert_eq!(
            node_path("car", "estimator", "aiding.gps.nis"),
            "agent.car.node.estimator.aiding.gps.nis"
        );
    }

    #[test]
    fn channel_path_spells_the_channel_by_its_name() {
        assert_eq!(
            channel_path("car", &channel_named::<f64>("oracle.pose")),
            "agent.car.channel.oracle.pose"
        );
    }

    #[test]
    fn split_takes_back_what_node_path_joined() {
        let path = node_path("car", "estimator", "aiding.gps.nis");

        assert_eq!(
            AgentPathParts::split(&path),
            Ok(AgentPathParts {
                agent: "car",
                subject: PathSubject::Node {
                    node: "estimator",
                    leaf: "aiding.gps.nis",
                },
            })
        );
    }

    #[test]
    fn split_takes_back_what_channel_path_joined() {
        let path = channel_path("car", &channel_named::<f64>("oracle.pose"));

        assert_eq!(
            AgentPathParts::split(&path),
            Ok(AgentPathParts {
                agent: "car",
                subject: PathSubject::Channel {
                    channel: "oracle.pose",
                },
            })
        );
    }

    #[test]
    fn subject_words_sit_where_a_node_would_never_collide() {
        let parts = AgentPathParts::split("agent.car.node.channel.speed").expect("node path");

        assert_eq!(
            parts.subject,
            PathSubject::Node {
                node: "channel",
                leaf: "speed",
            }
        );
    }

    #[test]
    fn split_refuses_a_path_outside_the_root() {
        assert_eq!(
            AgentPathParts::split("agents.car.node.estimator.nis"),
            Err(PathPart::Root)
        );
        assert_eq!(
            AgentPathParts::split("car.node.estimator.nis"),
            Err(PathPart::Root)
        );
    }

    #[test]
    fn split_refuses_a_missing_or_unknown_subject() {
        for path in [
            "agent.car",
            "agent.car..estimator.nis",
            "agent.car.estimator.nis",
            "agent.car.body.oracle.pose",
        ] {
            assert_eq!(
                AgentPathParts::split(path),
                Err(PathPart::Subject),
                "{path}"
            );
        }
    }

    #[test]
    fn split_names_the_first_missing_or_empty_part() {
        assert_eq!(AgentPathParts::split("agent"), Err(PathPart::Agent));
        assert_eq!(
            AgentPathParts::split("agent..node.estimator.nis"),
            Err(PathPart::Agent)
        );
        assert_eq!(AgentPathParts::split("agent.car.node"), Err(PathPart::Node));
        assert_eq!(
            AgentPathParts::split("agent.car.node..nis"),
            Err(PathPart::Node)
        );
        assert_eq!(
            AgentPathParts::split("agent.car.node.estimator"),
            Err(PathPart::Leaf)
        );
        assert_eq!(
            AgentPathParts::split("agent.car.node.estimator."),
            Err(PathPart::Leaf)
        );
        assert_eq!(
            AgentPathParts::split("agent.car.channel"),
            Err(PathPart::Channel)
        );
        assert_eq!(
            AgentPathParts::split("agent.car.channel."),
            Err(PathPart::Channel)
        );
    }
}
