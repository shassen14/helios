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
//! The spine leaf names ([`MAP`], [`ODOM`], [`BASE_LINK`]) are single-sourced here
//! and reused everywhere a producer and consumer must agree on them. A frame's
//! *role* (whether it is a localization root, an odometry origin, a body, a
//! sensor) is read back from the leaf via the predicates, never re-encoded in a
//! separate field.

use crate::kernel::identifier::LEVEL_SEPARATOR;
use crate::spatial::primitives::{AgentId, AGENT_SCOPE};

use std::fmt::Display;
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

    /// A sensor frame on the agent, named by the sensor's own config name (a leaf
    /// that is not part of the spine, e.g. `imu0`, `gps_antenna`).
    pub fn sensor(agent: AgentId, name: impl Into<Arc<str>>) -> Self {
        Self {
            scope: FrameScope::Agent(agent),
            leaf: FrameName(name.into()),
        }
    }

    /// Whether this frame is an agent's `map` root, regardless of which agent.
    pub fn is_map(&self) -> bool {
        self.is_agent_leaf(MAP)
    }

    /// Whether this frame is an agent's `odom` origin, regardless of which agent.
    pub fn is_odom(&self) -> bool {
        self.is_agent_leaf(ODOM)
    }

    /// Whether this frame is an agent's `base_link`, regardless of which agent.
    pub fn is_base_link(&self) -> bool {
        self.is_agent_leaf(BASE_LINK)
    }

    /// Whether this frame is a sensor frame: agent-scoped, but not one of the
    /// spine roles. A sensor's leaf is its own device name, so it is defined by
    /// exclusion — an agent frame that is neither `map`, `odom`, nor `base_link`
    /// is a sensor's own frame. A global frame is never a sensor.
    pub fn is_sensor(&self) -> bool {
        matches!(self.scope, FrameScope::Agent(_))
            && !self.is_map()
            && !self.is_odom()
            && !self.is_base_link()
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
    fn role_predicates_match_only_their_own_leaf() {
        assert!(FrameId::map(agent()).is_map());
        assert!(FrameId::odom(agent()).is_odom());
        assert!(FrameId::base_link(agent()).is_base_link());

        let base_link = FrameId::base_link(agent());
        assert!(!base_link.is_map());
        assert!(!base_link.is_odom());

        // A sensor leaf is on the spine's agent scope but holds no spine role.
        let sensor = FrameId::sensor(agent(), "imu0");
        assert!(!sensor.is_map());
        assert!(!sensor.is_odom());
        assert!(!sensor.is_base_link());

        // `is_sensor` is the exclusion of the spine roles within an agent's scope.
        assert!(sensor.is_sensor());
        assert!(!base_link.is_sensor());
        assert!(!FrameId::map(agent()).is_sensor());
        assert!(!FrameId::odom(agent()).is_sensor());
        // A global frame is never a sensor.
        assert!(!FrameId::global(DATUM).is_sensor());
    }

    #[test]
    fn a_global_frame_with_a_spine_leaf_holds_no_spine_role() {
        // Spine roles belong to an agent; a datum or fixture that happens to be
        // named like one must not be read as an agent's root or body.
        for leaf in [MAP, ODOM, BASE_LINK] {
            let frame = FrameId::global(leaf);
            assert!(!frame.is_map());
            assert!(!frame.is_odom());
            assert!(!frame.is_base_link());
            assert!(!frame.is_sensor());
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
            FrameId::sensor(agent(), "imu0").to_string(),
            "agent.pickup_1.imu0"
        );
    }

    #[test]
    fn display_keeps_dotted_leaves_whole() {
        // Everything after the fixed scope prefix is the leaf, so a dotted leaf
        // spells as its own levels and the spelling still splits back apart.
        assert_eq!(
            FrameId::sensor(agent(), "front.lidar").to_string(),
            "agent.pickup_1.front.lidar"
        );
        assert_eq!(
            FrameId::global("lbl.beacon_3").to_string(),
            "global.lbl.beacon_3"
        );
    }

    #[test]
    fn serde_round_trips() {
        for frame in [
            FrameId::global(DATUM),
            FrameId::map(agent()),
            FrameId::base_link(agent()),
            FrameId::sensor(agent(), "gps_antenna"),
        ] {
            let json = serde_json::to_string(&frame).expect("serialize");
            let back: FrameId = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(frame, back);
        }
    }
}
