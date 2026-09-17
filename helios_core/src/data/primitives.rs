use std::{fmt::Display, ops::Sub, sync::Arc};

use nalgebra::DVector;
use serde::{Deserialize, Serialize};

// --- Core Type Aliases ---
pub(crate) type State = DVector<f64>;
pub(crate) type Control = DVector<f64>;

/// Monotonically increasing time in seconds (simulation or hardware clock).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct MonotonicTime(pub f64);

impl Sub for MonotonicTime {
    type Output = MonotonicDuration;

    /// The elapsed interval between two instants. Signed: `earlier - later` is
    /// negative, which is how a lookup measures a query that falls before an
    /// edge's oldest sample.
    fn sub(self, rhs: MonotonicTime) -> MonotonicDuration {
        MonotonicDuration(self.0 - rhs.0)
    }
}

impl Sub<MonotonicDuration> for MonotonicTime {
    type Output = MonotonicTime;

    /// Shifts an instant back by a duration — e.g. `newest - horizon` yields the
    /// eviction cutoff, the oldest stamp a bounded history still keeps.
    fn sub(self, rhs: MonotonicDuration) -> MonotonicTime {
        MonotonicTime(self.0 - rhs.0)
    }
}

/// A signed interval in seconds — the difference of two [`MonotonicTime`]s.
///
/// Signed and `f64` because simulation time is a monotonic `f64`, not a
/// wall-clock instant; `std::time::Duration` is the wrong type here (it is
/// unsigned and wall-clock-flavoured, and a query *before* an edge's history
/// yields a legitimately negative interval). The name says nothing about sim vs
/// hardware because the whole frame stack runs identically on both.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct MonotonicDuration(pub f64);

/// Stable string identity of one agent, shared by every subsystem that names it
/// (frame scope, determinism seeding, observability paths) — a reserved string
/// whose sides must agree. Backed by `Arc<str>` so cloning it (onto a `FrameId`,
/// for instance) is cheap; it serializes as (and loads from) a plain string.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct AgentId(Arc<str>);

impl AgentId {
    pub fn new(id: impl Into<Arc<str>>) -> Self {
        Self(id.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl Display for AgentId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl Serialize for AgentId {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for AgentId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Ok(AgentId::new(String::deserialize(deserializer)?))
    }
}
