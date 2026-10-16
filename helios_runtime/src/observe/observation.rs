//! A value recorded for watchers (tests, visualizers, recorders): what a
//! node emitted about itself, or a value written to a channel.
//!
//! Observations leave the brain and are never read back by it; the bus
//! carries what nodes consume. A node emits raw samples, and whoever watches
//! computes statistics over them (a mean, a settling time, a rate across
//! runs), so the node never fixes which summaries exist.

use crate::{observe::key::RecordedKey, ErasedStamped};

use helios_core::prelude::MonotonicTime;

use std::{any::TypeId, fmt::Debug, sync::Arc};

/// One recorded value, with the key of the series it belongs to: what a
/// pipeline drains and a host hands its sinks.
///
/// The key names the agent, so a host draining several pipelines into one
/// batch adds nothing. Fields are public so sinks can read them and
/// host-side scorers can build their own observations; every field is
/// shared, so a sink keeps a sample by cloning it cheaply.
#[derive(Debug, Clone, PartialEq)]
pub struct Observation {
    pub key: RecordedKey,
    /// The time the value describes, not when it was emitted: a measurement
    /// update stamps the reading's time.
    pub timestamp: MonotonicTime,
    pub value: ObservedValue,
}

/// The kinds of value an observation can carry.
///
/// Non-exhaustive: richer kinds (structured events) are added as variants, so
/// a `match` outside this crate needs a wildcard arm, and a sink skips kinds
/// it doesn't handle. Not `Copy`, since [`Typed`](Self::Typed) holds shared
/// data.
#[derive(Clone)]
#[non_exhaustive]
pub enum ObservedValue {
    /// A number in SI units: a statistic such as NIS, a count (each event
    /// emits `1.0`), or a duration in seconds.
    Scalar(f64),
    /// A value written to a channel: the bus slot's own `Arc`, shared rather
    /// than copied. The observation's timestamp is the
    /// value's own. Two are equal only when they are the same write, never by
    /// comparing payloads.
    Typed(Arc<dyn ErasedStamped>),
}

impl ObservedValue {
    /// Which kind of value this is.
    pub fn kind(&self) -> ObservedValueKind {
        // No wildcard: a new variant is a compile error here until it says
        // which kind it is, so the two enums can't drift apart.
        match self {
            Self::Scalar(_) => ObservedValueKind::Scalar,
            Self::Typed(stamped) => ObservedValueKind::Typed {
                type_id: stamped.payload_type_id(),
                type_name: stamped.payload_type_name(),
            },
        }
    }
}

impl Debug for ObservedValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Scalar(value) => f.debug_tuple("Scalar").field(value).finish(),
            Self::Typed(stamped) => f
                .debug_struct("Typed")
                .field("payload", &stamped.payload_type_name())
                .field("timestamp", &stamped.timestamp())
                .finish(),
        }
    }
}

impl PartialEq for ObservedValue {
    fn eq(&self, other: &Self) -> bool {
        // Matching on `self` alone, with no wildcard, makes a new variant a
        // compile error here until it says how it compares.
        match self {
            Self::Scalar(mine) => matches!(other, Self::Scalar(theirs) if mine == theirs),
            Self::Typed(mine) => {
                matches!(other, Self::Typed(theirs) if Arc::ptr_eq(mine, theirs))
            }
        }
    }
}

impl From<f64> for ObservedValue {
    fn from(value: f64) -> Self {
        ObservedValue::Scalar(value)
    }
}

/// The kind of an [`ObservedValue`], without the value: what a leaf promises
/// before anything arrives, as its [`Observable`](crate::port::Observable)
/// records.
///
/// So a watcher can check, when it asks for a leaf, that it can read what
/// will arrive there: an assertion that it can extract a comparable value, a
/// viewer that it has a renderer for the payload.
/// [`ObservedValue::kind`] maps each value to its kind, so a new
/// `ObservedValue` variant doesn't compile until it has one here.
///
/// Exhaustive, unlike `ObservedValue`: a new kind must make every reader's
/// `match` decide what to do with it, before a value of it can arrive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ObservedValueKind {
    /// Arrives as [`ObservedValue::Scalar`].
    Scalar,
    /// Arrives as [`ObservedValue::Typed`], with a payload of this type.
    Typed {
        type_id: TypeId,
        type_name: &'static str,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::prelude::{Health, Stamped};

    #[test]
    fn a_scalar_is_of_the_scalar_kind() {
        assert_eq!(ObservedValue::Scalar(0.5).kind(), ObservedValueKind::Scalar);
    }

    #[test]
    fn a_typed_value_is_of_its_payloads_kind() {
        let stamped: Arc<dyn ErasedStamped> = Arc::new(Stamped {
            value: 7_u32,
            timestamp: MonotonicTime(0.0),
            health: Health::Ok,
            producer: 0,
        });

        assert_eq!(
            ObservedValue::Typed(stamped).kind(),
            ObservedValueKind::Typed {
                type_id: TypeId::of::<u32>(),
                type_name: std::any::type_name::<u32>(),
            }
        );
    }
}
