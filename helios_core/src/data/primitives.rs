use std::{fmt::Display, sync::Arc};

use nalgebra::DVector;
use serde::{Deserialize, Serialize};

// --- Core Type Aliases ---
pub(crate) type State = DVector<f64>;
pub(crate) type Control = DVector<f64>;

/// Monotonically increasing time in seconds (simulation or hardware clock).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct MonotonicTime(pub f64);

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
