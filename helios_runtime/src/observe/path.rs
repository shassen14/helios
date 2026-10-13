//! How names are spelled inside a watcher path.
//!
//! Watchers address everything recorded about an agent by a dotted path,
//! `agent.<agent>.<subject>.…`. The subject word says what kind of value the
//! path names, and decides what follows it:
//!
//! - `node.<node>.<leaf>`: what a pipeline node said about itself, under a
//!   leaf it declared, or a value it wrote to one of its output channels.
//! - `channel.<instance>`: a value on one of the agent's channels, whoever
//!   wrote it.
//!
//! Who wrote a value is not part of its path; the path names only what the
//! value is, so swapping the writer (one estimator for another, a host for a
//! replay) keeps every path. The subject words are fixed and sit in a part no
//! config fills, so a name a profile chooses never collides with one. The
//! agent and the node each fill exactly one part, so their names must be
//! [path segments](is_path_segment). A node's leaves and its output channels'
//! segments, built here, share the part after the node, so the build compares
//! the two.

use crate::port::ChannelKey;

/// The one level separator in watcher paths.
pub(crate) const PATH_SEPARATOR: &str = ".";

/// The first segment of every watcher path.
pub(crate) const AGENT_ROOT: &str = "agent";

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
    /// each end them, and everything after is the leaf or the instance. On
    /// failure, names the first part that is wrong: a root other than
    /// [`AGENT_ROOT`], a subject word that isn't one of [`SUBJECT_WORDS`],
    /// or a part that is missing or empty.
    pub fn split(path: &'a str) -> Result<Self, PathPart> {
        let mut parts = path.splitn(4, PATH_SEPARATOR);
        if parts.next() != Some(AGENT_ROOT) {
            return Err(PathPart::Root);
        }
        let agent = non_empty(parts.next()).ok_or(PathPart::Agent)?;
        let word = parts.next().ok_or(PathPart::Subject)?;
        let after = parts.next();
        let subject = match word {
            NODE_SUBJECT => {
                let mut node_parts = after.unwrap_or_default().splitn(2, PATH_SEPARATOR);
                let node = non_empty(node_parts.next()).ok_or(PathPart::Node)?;
                let leaf = non_empty(node_parts.next()).ok_or(PathPart::Leaf)?;
                PathSubject::Node { node, leaf }
            }
            CHANNEL_SUBJECT => PathSubject::Channel {
                instance: non_empty(after).ok_or(PathPart::Instance)?,
            },
            _ => return Err(PathPart::Subject),
        };

        Ok(Self { agent, subject })
    }

    /// The watcher path these parts spell. The agent and a node must each be
    /// a [path segment](is_path_segment); the build refuses names that
    /// aren't. A leaf and an instance may hold dots: a leaf a node declared
    /// (`aiding.gps.nis`) or a channel's segment (`oracle.pose`).
    pub fn join(&self) -> String {
        let mut parts = vec![AGENT_ROOT, self.agent, self.subject.word()];
        match self.subject {
            PathSubject::Node { node, leaf } => parts.extend([node, leaf]),
            PathSubject::Channel { instance } => parts.push(instance),
        }
        parts.join(PATH_SEPARATOR)
    }
}

/// What kind of value a watcher path names, with the parts its path holds
/// after the subject word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathSubject<'a> {
    /// What a pipeline node said about itself, or wrote to an output channel.
    Node {
        node: &'a str,
        /// The leaf or output channel segment, dots included.
        leaf: &'a str,
    },
    /// A value on one of the agent's channels, spelled by its
    /// [segment](channel_to_path_segment).
    Channel { instance: &'a str },
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
    /// The node's leaf or output channel segment.
    Leaf,
    /// The channel's segment, under the `channel` subject.
    Instance,
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
/// `agent.<agent>.channel.<segment>`. [`AgentPathParts::split`] takes it
/// apart again.
pub fn channel_path(agent: &str, channel: &ChannelKey) -> String {
    AgentPathParts {
        agent,
        subject: PathSubject::Channel {
            instance: &channel_to_path_segment(channel),
        },
    }
    .join()
}

/// `part` unless it is missing or empty.
fn non_empty(part: Option<&str>) -> Option<&str> {
    part.filter(|part| !part.is_empty())
}

/// Whether `name` can fill exactly one part of a watcher path: it is not
/// empty and holds no [`PATH_SEPARATOR`]. Agent and node names must be; an
/// empty name or a dotted one would make the path impossible to split back
/// into its parts.
pub(crate) fn is_path_segment(name: &str) -> bool {
    !name.is_empty() && !name.contains(PATH_SEPARATOR)
}

/// The path segment for `key`: its instance, else its type's name.
///
/// The instance is preferred because it is the name a person wrote in the
/// config. Each `/` in it becomes `.`, so `oracle/pose` sits at
/// `oracle.pose`: `.` is the one level separator in paths, and `_` stays a
/// word joiner. An unnamed channel falls back to the last part of its type
/// name in snake_case (`FrameAwareState` → `frame_aware_state`).
pub(crate) fn channel_to_path_segment(key: &ChannelKey) -> String {
    let raw = key.instance();
    if !raw.trim().is_empty() {
        raw.replace('/', PATH_SEPARATOR)
    } else {
        to_snake_case(last_segment(key.type_name()))
    }
}

/// The part of a type name after its last `::`.
fn last_segment(type_name: &'static str) -> &'static str {
    type_name.rsplit("::").next().unwrap_or(type_name)
}

/// Lowercase, with `_` before each uppercase letter after the first:
/// `FrameAwareState` → `frame_aware_state`, `f64` → `f64`. Enough for Rust
/// type names; not a general snake_case (no digit boundaries, no acronym
/// handling).
fn to_snake_case(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 4);
    for (i, ch) in s.chars().enumerate() {
        if i > 0 && ch.is_uppercase() {
            out.push('_');
        }
        out.extend(ch.to_lowercase());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::InternalChannel;

    fn channel_named<T: 'static>(instance: &'static str) -> ChannelKey {
        InternalChannel::named::<T>(instance).into()
    }

    fn channel_unnamed<T: 'static>() -> ChannelKey {
        InternalChannel::of::<T>().into()
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
    fn channel_path_spells_the_channel_by_its_segment() {
        assert_eq!(
            channel_path("car", &channel_named::<f64>("oracle/pose")),
            "agent.car.channel.oracle.pose"
        );
        assert_eq!(
            channel_path("car", &channel_unnamed::<f64>()),
            "agent.car.channel.f64"
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
        let path = channel_path("car", &channel_named::<f64>("oracle/pose"));

        assert_eq!(
            AgentPathParts::split(&path),
            Ok(AgentPathParts {
                agent: "car",
                subject: PathSubject::Channel {
                    instance: "oracle.pose",
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
            Err(PathPart::Instance)
        );
        assert_eq!(
            AgentPathParts::split("agent.car.channel."),
            Err(PathPart::Instance)
        );
    }

    #[test]
    fn plain_name_is_a_path_segment() {
        assert!(is_path_segment("primary"));
        assert!(is_path_segment("front_lidar_deproject"));
    }

    #[test]
    fn dotted_or_empty_name_is_not_a_path_segment() {
        assert!(!is_path_segment("a.b"));
        assert!(!is_path_segment(".a"));
        assert!(!is_path_segment(""));
    }

    #[test]
    fn channel_to_path_segment_uses_instance() {
        let key = channel_named::<f64>("oracle/pose");
        assert_eq!(channel_to_path_segment(&key), "oracle.pose");
    }

    #[test]
    fn channel_to_path_segment_replaces_all_slashes() {
        let key = channel_named::<f64>("debug/inner/value");
        assert_eq!(channel_to_path_segment(&key), "debug.inner.value");
    }

    #[test]
    fn channel_to_path_segment_keeps_dots_and_underscores() {
        let key = channel_named::<f64>("sensor.gps.front_left");
        assert_eq!(channel_to_path_segment(&key), "sensor.gps.front_left");
    }

    #[test]
    fn channel_to_path_segment_falls_back_to_type_name() {
        let key = channel_unnamed::<f64>();
        assert_eq!(channel_to_path_segment(&key), "f64");
    }

    #[test]
    fn last_segment_strips_module_path() {
        assert_eq!(last_segment("foo::bar::Baz"), "Baz");
        assert_eq!(last_segment("Baz"), "Baz");
        assert_eq!(last_segment("f64"), "f64");
    }

    #[test]
    fn to_snake_case_basic_cases() {
        assert_eq!(to_snake_case("FrameAwareState"), "frame_aware_state");
        assert_eq!(to_snake_case("f64"), "f64");
        assert_eq!(to_snake_case("A"), "a");
        assert_eq!(to_snake_case(""), "");
        assert_eq!(to_snake_case("MyType"), "my_type");
    }
}
