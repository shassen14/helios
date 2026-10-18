//! Keyed payloads: one channel carrying many things, the last value of each.
//!
//! A payload type implementing [`Keyed`] names, from its own fields, which
//! thing a value is about: a transform names its child frame. A channel
//! declared [`keyed`](crate::port::ChannelKey::keyed) keeps the last value
//! per [`InstanceKey`] rather than one value for the whole channel, in
//! instance order so draining it is deterministic. Because the instance is
//! read off the value, a writer cannot put a value on another instance than
//! the one it is about.
//!
//! Each writer declares the instances it writes, and the pipeline build
//! refuses two writers of one instance. The bus refuses a write of an
//! instance no writer declared.

use std::{any::Any, fmt::Display, sync::Arc};

/// A payload whose values each belong to one instance of their channel.
pub trait Keyed: Any + Send + Sync {
    /// What tells the channel's instances apart. Its `Display` is the
    /// instance's spelling, the one form the bus, recordings and transports
    /// share.
    type Instance: Display;

    /// The instance this value belongs to, read off its own fields.
    fn instance(&self) -> &Self::Instance;
}

/// One instance of a keyed channel, by its spelling.
///
/// Ordered by spelling, so a keyed channel's instances drain in the same
/// order in every run.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct InstanceKey(Arc<str>);

impl InstanceKey {
    /// The key of `instance` of a `T` channel, spelled as `T`'s instances
    /// are.
    pub fn of<T: Keyed>(instance: &T::Instance) -> Self {
        Self(instance.to_string().into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl Display for InstanceKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Track {
        id: u32,
    }

    impl Keyed for Track {
        type Instance = u32;

        fn instance(&self) -> &u32 {
            &self.id
        }
    }

    #[test]
    fn an_instance_is_keyed_by_its_spelling() {
        let track = Track { id: 7 };
        assert_eq!(InstanceKey::of::<Track>(track.instance()).as_str(), "7");
        assert_eq!(InstanceKey::of::<Track>(&7).to_string(), "7");
    }

    #[test]
    fn instances_order_by_spelling() {
        let mut keys = [
            InstanceKey::of::<Track>(&2),
            InstanceKey::of::<Track>(&10),
            InstanceKey::of::<Track>(&1),
        ];
        keys.sort();
        let spelled: Vec<&str> = keys.iter().map(InstanceKey::as_str).collect();
        assert_eq!(spelled, ["1", "10", "2"]);
    }
}
