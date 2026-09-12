//! Coordinate-frame identity: a frame is named by *scope* and *leaf*.
//!
//! [`FrameId`] answers "which coordinate frame is this?" without reference to any
//! ECS entity or runtime handle, so the same identifier is valid in simulation and
//! on hardware. Identity has two parts:
//!
//! - [`FrameScope`] — the shared `World` frame, or one specific agent's private
//!   frame tree. Two agents each own a `base_link`; the scope is what keeps them
//!   distinct (`agent_a/base_link != agent_b/base_link`).
//! - [`FrameName`] — the leaf name within that scope (`base_link`, `odom`, a
//!   sensor's own name). Held as an `Arc<str>` so a `FrameId` is cheap to clone
//!   and hash while carrying a human-readable, wire-compatible name.
//!
//! The spine leaf names ([`MAP`], [`ODOM`], [`BASE_LINK`]) are single-sourced here
//! and reused everywhere a producer and consumer must agree on them. A frame's
//! *role* (whether it is a localization root, an odometry origin, a body, a
//! sensor) is read back from the leaf via the predicates, never re-encoded in a
//! separate field.

use crate::data::AgentId;

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

/// A hashable, clone-cheap identifier for one coordinate frame.
///
/// Two `FrameId`s are equal iff both their scope and leaf match, so an agent's
/// frames never collide with the shared world frame or with another agent's.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct FrameId {
    scope: FrameScope,
    leaf: FrameName,
}

impl FrameId {
    /// The shared global frame. Its leaf is empty — `World` is identified by scope
    /// alone, and [`Display`] renders it as `world`.
    pub fn world() -> Self {
        Self {
            scope: FrameScope::World,
            leaf: FrameName(Arc::from("")),
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
        self.leaf.as_str() == MAP
    }

    /// Whether this frame is an agent's `odom` origin, regardless of which agent.
    pub fn is_odom(&self) -> bool {
        self.leaf.as_str() == ODOM
    }

    /// Whether this frame is an agent's `base_link`, regardless of which agent.
    pub fn is_base_link(&self) -> bool {
        self.leaf.as_str() == BASE_LINK
    }

    /// Whether this frame is a sensor frame: agent-scoped, but not one of the
    /// spine roles. A sensor's leaf is its own device name, so it is defined by
    /// exclusion — an agent frame that is neither `map`, `odom`, nor `base_link`
    /// is a sensor's own frame. The shared `World` frame is never a sensor.
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
}

impl Display for FrameId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.scope {
            FrameScope::World => f.write_str("world"),
            FrameScope::Agent(id) => write!(f, "{}/{}", id, self.leaf.as_str()),
        }
    }
}

/// Which frame tree a [`FrameId`] belongs to: the shared world, or one agent's own.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FrameScope {
    World,
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

    use crate::data::AgentId;

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
        // The shared world frame is never a sensor.
        assert!(!FrameId::world().is_sensor());
    }

    #[test]
    fn scope_distinguishes_agents_and_the_world() {
        let a = FrameId::base_link(AgentId::new("a"));
        let b = FrameId::base_link(AgentId::new("b"));

        // Same leaf, different scope -> different frame.
        assert_ne!(a, b);
        assert_eq!(a, FrameId::base_link(AgentId::new("a")));

        assert!(matches!(FrameId::world().scope(), FrameScope::World));
        assert!(matches!(a.scope(), FrameScope::Agent(_)));
    }

    #[test]
    fn display_renders_the_wire_form() {
        assert_eq!(FrameId::world().to_string(), "world");
        assert_eq!(
            FrameId::base_link(agent()).to_string(),
            "pickup_1/base_link"
        );
        assert_eq!(
            FrameId::sensor(agent(), "imu0").to_string(),
            "pickup_1/imu0"
        );
    }

    #[test]
    fn serde_round_trips() {
        for frame in [
            FrameId::world(),
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
