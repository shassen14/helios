//! Coordinate-frame identity: a frame is named by *scope* and *leaf*.
//!
//! [`FrameId`] answers "which coordinate frame is this?" without reference to any
//! ECS entity or runtime handle, so the same identifier is valid in simulation and
//! on hardware. Identity has two parts:
//!
//! - [`FrameScope`] — a global frame no agent owns, or one specific agent's
//!   private frame tree. Two agents each own a `base_link`; the scope is what
//!   keeps them distinct (`agent.a.base_link != agent.b.base_link`).
//! - [`FrameName`] — the leaf name within that scope (`base_link`, `odom`, a
//!   sensor's own name, a datum's name). Held as an `Arc<str>` so a `FrameId` is
//!   cheap to clone and hash while carrying a human-readable, wire-compatible
//!   name.
//!
//! A global frame is any frame no agent owns: a datum (a fixed origin a site or
//! world declares), a surveyed fixture, a celestial frame. Each is named, so two
//! sites' origins are two frames and nothing joins them unless an edge does.
//!
//! A frame's spelling (`agent.<agent>.<leaf>`, `global.<leaf>`) is its
//! `Display`, and `FromStr` parses it back, so a frame named in a run file, a
//! recording or a bridge from another stack is parsed once, at that edge, and
//! refused there when it isn't one.
//!
//! The spine leaf names ([`MAP`], [`ODOM`], [`BASE_LINK`]) are single-sourced here
//! and reused everywhere a producer and consumer must agree on them. A frame's
//! *role* (whether it is a localization root, an odometry origin, a body, a
//! sensor) is read back from the leaf via the predicates, never re-encoded in a
//! separate field.

use crate::kernel::identifier::{is_dotted_name, is_level, LEVEL_SEPARATOR};
use crate::spatial::primitives::{AgentId, AGENT_SCOPE};

use std::error::Error;
use std::fmt::Display;
use std::str::FromStr;
use std::sync::Arc;

use serde::{Deserialize, Serialize};

/// Leaf name of a per-agent localization / SLAM root. The `map -> odom` edge above
/// it is where a localizer removes odometry drift and may jump on loop closure.
pub const MAP: &str = "map";
/// Leaf name of a per-agent odometry origin. The `odom -> base_link` edge below it
/// is smooth and continuous — the frame controllers and planners read against.
pub const ODOM: &str = "odom";
/// Leaf name of the agent's body root — the fixed calibration datum every sensor
/// extrinsic and the estimated pose are expressed against.
pub const BASE_LINK: &str = "base_link";

/// The first level of a global frame's spelling (`global.<leaf>`).
pub const GLOBAL_SCOPE: &str = "global";

/// A hashable, clone-cheap identifier for one coordinate frame.
///
/// Two `FrameId`s are equal iff both their scope and leaf match, so an agent's
/// frames never collide with a global frame or with another agent's.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct FrameId {
    scope: FrameScope,
    leaf: FrameName,
}

impl FrameId {
    /// A global frame, named by its leaf (a datum, a surveyed fixture).
    pub fn global(leaf: impl Into<Arc<str>>) -> Self {
        Self {
            scope: FrameScope::Global,
            leaf: FrameName(leaf.into()),
        }
    }

    /// The agent's localization / SLAM root (`map`).
    pub fn map(agent: AgentId) -> Self {
        Self {
            scope: FrameScope::Agent(agent),
            leaf: FrameName(Arc::from(MAP)),
        }
    }

    /// The agent's odometry origin (`odom`).
    pub fn odom(agent: AgentId) -> Self {
        Self {
            scope: FrameScope::Agent(agent),
            leaf: FrameName(Arc::from(ODOM)),
        }
    }

    /// The agent's body root (`base_link`).
    pub fn base_link(agent: AgentId) -> Self {
        Self {
            scope: FrameScope::Agent(agent),
            leaf: FrameName(Arc::from(BASE_LINK)),
        }
    }

    /// A frame on the agent's body, named by its leaf (a sensor mount, a wheel
    /// hub, an arm link; e.g. `imu0`, `gps_antenna`, `wheel_fl`).
    pub fn agent(agent: AgentId, name: impl Into<Arc<str>>) -> Self {
        Self {
            scope: FrameScope::Agent(agent),
            leaf: FrameName(name.into()),
        }
    }

    /// Whether this frame is an agent's `base_link`, regardless of which agent.
    pub fn is_base_link(&self) -> bool {
        self.is_agent_leaf(BASE_LINK)
    }

    pub fn scope(&self) -> &FrameScope {
        &self.scope
    }

    pub fn leaf(&self) -> &FrameName {
        &self.leaf
    }

    /// Whether this is an agent's frame with the given leaf. A spine role is an
    /// agent's, so a global frame that happens to share a spine leaf holds no
    /// role.
    fn is_agent_leaf(&self, leaf: &str) -> bool {
        matches!(self.scope, FrameScope::Agent(_)) && self.leaf.as_str() == leaf
    }
}

/// Spells the frame as levels: `agent.<agent>.<leaf>` or `global.<leaf>`.
///
/// The scope word comes first, where no config name goes, so no agent or frame
/// name is reserved. The spelling inverts because an agent name is one level:
/// whatever follows `agent.<agent>.` or `global.` is the leaf.
impl Display for FrameId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let sep = LEVEL_SEPARATOR;
        let leaf = self.leaf.as_str();
        match &self.scope {
            FrameScope::Global => write!(f, "{GLOBAL_SCOPE}{sep}{leaf}"),
            FrameScope::Agent(id) => write!(f, "{AGENT_SCOPE}{sep}{id}{sep}{leaf}"),
        }
    }
}

/// Parses the spelling [`Display`] writes, and only that: the scope word, the
/// agent when the scope is an agent's, then the leaf, which is everything
/// after them. Each name is checked against the level grammar, so a parsed
/// frame always spells back to the text it came from.
impl FromStr for FrameId {
    type Err = FrameSpellingError;

    fn from_str(spelling: &str) -> Result<Self, Self::Err> {
        let (scope_word, rest) = split_first_level(spelling);
        let (scope, leaf) = match scope_word {
            GLOBAL_SCOPE => (FrameScope::Global, rest),
            AGENT_SCOPE => {
                let (agent, leaf) = split_first_level(rest);
                if !is_level(agent) {
                    return Err(FrameSpellingError::MalformedAgent);
                }
                (FrameScope::Agent(AgentId::new(agent)), leaf)
            }
            _ => return Err(FrameSpellingError::UnknownScope),
        };
        if leaf.is_empty() {
            return Err(FrameSpellingError::NoLeaf);
        }
        if !is_dotted_name(leaf) {
            return Err(FrameSpellingError::MalformedLeaf);
        }
        Ok(Self {
            scope,
            leaf: FrameName(Arc::from(leaf)),
        })
    }
}

/// The text before the first level separator and the text after it; the
/// whole text and nothing when there is no separator.
fn split_first_level(text: &str) -> (&str, &str) {
    text.split_once(LEVEL_SEPARATOR).unwrap_or((text, ""))
}

/// Why a spelling is not a [`FrameId`]'s. The caller holds the spelling, so
/// it names it beside this.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameSpellingError {
    /// The first level is neither scope word: a bare leaf such as another
    /// stack's `base_link`, or a misspelled scope.
    UnknownScope,
    /// The agent after the agent scope word is not one level.
    MalformedAgent,
    /// Nothing follows the scope (and agent): no frame is named.
    NoLeaf,
    /// The leaf is not levels joined by the level separator.
    MalformedLeaf,
}

impl Display for FrameSpellingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let sep = LEVEL_SEPARATOR;
        match self {
            Self::UnknownScope => write!(
                f,
                "its first level is neither '{AGENT_SCOPE}' nor '{GLOBAL_SCOPE}'"
            ),
            Self::MalformedAgent => write!(
                f,
                "its agent is not one level of ASCII letters, digits and '_'"
            ),
            Self::NoLeaf => write!(f, "it names no frame after its scope"),
            Self::MalformedLeaf => write!(
                f,
                "its leaf is not levels of ASCII letters, digits and '_' joined by '{sep}'"
            ),
        }
    }
}

impl Error for FrameSpellingError {}

/// Which frame tree a [`FrameId`] belongs to: the global frames no agent owns,
/// or one agent's own.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FrameScope {
    Global,
    Agent(AgentId),
}

/// The leaf name of a frame within its scope. Wraps an `Arc<str>` so the name is
/// carried by reference-count, not copied, on every clone of a [`FrameId`]; it
/// serializes as (and loads from) a plain string.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct FrameName(Arc<str>);

impl FrameName {
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl Serialize for FrameName {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for FrameName {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Ok(FrameName(String::deserialize(deserializer)?.into()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::prelude::AgentId;

    const DATUM: &str = "test_datum";

    fn agent() -> AgentId {
        AgentId::new("pickup_1")
    }

    #[test]
    fn spine_constructors_use_distinct_leaves() {
        // Guards against the copy-paste class of bug where two spine frames share a
        // leaf and silently collapse into one identity.
        let map = FrameId::map(agent());
        let odom = FrameId::odom(agent());
        let base_link = FrameId::base_link(agent());

        assert_eq!(map.leaf().as_str(), MAP);
        assert_eq!(odom.leaf().as_str(), ODOM);
        assert_eq!(base_link.leaf().as_str(), BASE_LINK);

        assert_ne!(map, odom);
        assert_ne!(odom, base_link);
        assert_ne!(map, base_link);
    }

    #[test]
    fn the_base_link_predicate_matches_only_its_own_leaf() {
        assert!(FrameId::base_link(agent()).is_base_link());
        assert!(!FrameId::odom(agent()).is_base_link());

        // A sensor leaf is on the spine's agent scope but holds no spine role.
        assert!(!FrameId::agent(agent(), "imu0").is_base_link());
    }

    #[test]
    fn a_global_frame_with_a_spine_leaf_holds_no_spine_role() {
        // Spine roles belong to an agent; a datum or fixture that happens to be
        // named like one must not be read as an agent's root or body.
        for leaf in [MAP, ODOM, BASE_LINK] {
            assert!(!FrameId::global(leaf).is_base_link());
        }
    }

    #[test]
    fn scope_distinguishes_agents_and_global_frames() {
        let a = FrameId::base_link(AgentId::new("a"));
        let b = FrameId::base_link(AgentId::new("b"));

        // Same leaf, different scope -> different frame.
        assert_ne!(a, b);
        assert_eq!(a, FrameId::base_link(AgentId::new("a")));

        assert!(matches!(FrameId::global(DATUM).scope(), FrameScope::Global));
        // Same leaf, global vs agent scope -> different frame.
        assert_ne!(FrameId::global(BASE_LINK), a);
        assert!(matches!(a.scope(), FrameScope::Agent(_)));
    }

    #[test]
    fn display_spells_scope_then_levels() {
        assert_eq!(FrameId::global(DATUM).to_string(), "global.test_datum");
        assert_eq!(
            FrameId::base_link(agent()).to_string(),
            "agent.pickup_1.base_link"
        );
        assert_eq!(
            FrameId::agent(agent(), "imu0").to_string(),
            "agent.pickup_1.imu0"
        );
    }

    #[test]
    fn display_keeps_dotted_leaves_whole() {
        // Everything after the fixed scope prefix is the leaf, so a dotted leaf
        // spells as its own levels and the spelling still splits back apart.
        assert_eq!(
            FrameId::agent(agent(), "front.lidar").to_string(),
            "agent.pickup_1.front.lidar"
        );
        assert_eq!(
            FrameId::global("lbl.beacon_3").to_string(),
            "global.lbl.beacon_3"
        );
    }

    #[test]
    fn parsing_inverts_display_in_every_scope() {
        for frame in [
            FrameId::global(DATUM),
            FrameId::global("lbl.beacon_3"),
            FrameId::map(agent()),
            FrameId::odom(agent()),
            FrameId::base_link(agent()),
            FrameId::agent(agent(), "imu0"),
            FrameId::agent(agent(), "front.lidar"),
        ] {
            let spelling = frame.to_string();
            assert_eq!(spelling.parse::<FrameId>(), Ok(frame), "{spelling}");
        }
    }

    #[test]
    fn a_dotted_leaf_is_the_remainder_after_the_scope() {
        let frame: FrameId = "agent.pickup_1.front.lidar".parse().expect("a frame");

        assert_eq!(frame.scope(), &FrameScope::Agent(agent()));
        assert_eq!(frame.leaf().as_str(), "front.lidar");
    }

    #[test]
    fn a_spelling_that_isnt_a_frames_is_refused_saying_why() {
        for (spelling, why) in [
            // Another stack's frames name no scope.
            ("base_link", FrameSpellingError::UnknownScope),
            ("odom", FrameSpellingError::UnknownScope),
            ("/base_link", FrameSpellingError::UnknownScope),
            (
                "agents.pickup_1.base_link",
                FrameSpellingError::UnknownScope,
            ),
            ("", FrameSpellingError::UnknownScope),
            ("agent", FrameSpellingError::MalformedAgent),
            ("agent..base_link", FrameSpellingError::MalformedAgent),
            (
                "agent.pickup-1.base_link",
                FrameSpellingError::MalformedAgent,
            ),
            ("agent.pickup_1", FrameSpellingError::NoLeaf),
            ("agent.pickup_1.", FrameSpellingError::NoLeaf),
            ("global", FrameSpellingError::NoLeaf),
            ("global.", FrameSpellingError::NoLeaf),
            (
                "agent.pickup_1.front..lidar",
                FrameSpellingError::MalformedLeaf,
            ),
            (
                "agent.pickup_1.base_link.",
                FrameSpellingError::MalformedLeaf,
            ),
            ("global.test datum", FrameSpellingError::MalformedLeaf),
        ] {
            assert_eq!(spelling.parse::<FrameId>(), Err(why), "'{spelling}'");
        }
    }

    #[test]
    fn serde_round_trips() {
        for frame in [
            FrameId::global(DATUM),
            FrameId::map(agent()),
            FrameId::base_link(agent()),
            FrameId::agent(agent(), "gps_antenna"),
        ] {
            let json = serde_json::to_string(&frame).expect("serialize");
            let back: FrameId = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(frame, back);
        }
    }
}
