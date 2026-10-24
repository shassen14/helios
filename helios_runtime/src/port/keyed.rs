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
//! instance no writer declared. A payload whose values are expressed in a
//! frame other than their instance, as a transform is in its parent, is
//! declared with that frame, so a reader knows it before the first value,
//! and the bus refuses a value expressed in another.

use helios_core::spatial::FrameId;

use std::{any::Any, collections::BTreeMap, fmt::Display, sync::Arc};

/// A payload whose values each belong to one instance of their channel.
pub trait Keyed: Any + Send + Sync {
    /// What tells the channel's instances apart. Its `Display` is the
    /// instance's spelling, the one form the bus, recordings and transports
    /// share.
    type Instance: Display;

    /// What a writer declares of each instance it writes: the instance
    /// itself, or, for a payload expressed in another frame, the instance
    /// with that frame (a transform's edge, child and parent), so it can't
    /// be declared without it.
    type Declared: InstanceDeclaration<Self::Instance>;

    /// The instance this value belongs to, read off its own fields.
    fn instance(&self) -> &Self::Instance;

    /// The frame this value is expressed in when `Declared` names one for
    /// its instance (a transform's parent), read off its own fields; `None`
    /// when the instance is declared bare, even if each value names frames
    /// of its own. The bus refuses a value whose frame differs from its
    /// instance's declaration.
    fn declared_frame(&self) -> Option<&FrameId>;
}

/// What a writer declares of one instance `I` of a keyed channel.
pub trait InstanceDeclaration<I> {
    /// The instance it declares.
    fn declares(&self) -> &I;

    /// The frame the instance's values are expressed in, when the payload
    /// names one apart from the instance.
    fn expressed_in(&self) -> Option<&FrameId>;
}

/// An instance declared bare: its values name no other frame.
impl<I> InstanceDeclaration<I> for I {
    fn declares(&self) -> &I {
        self
    }

    fn expressed_in(&self) -> Option<&FrameId> {
        None
    }
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

/// The instances one writer declares of a keyed channel, in declaration
/// order, each with the frame its values are expressed in when the payload
/// names one. Empty for a channel that holds one value.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeclaredInstances {
    keys: Vec<InstanceKey>,
    expressed_in: BTreeMap<InstanceKey, FrameId>,
}

impl DeclaredInstances {
    /// The instances `declared` of a `T` channel, each spelled as `T`'s
    /// instances are.
    pub fn of<T: Keyed>(declared: impl IntoIterator<Item = T::Declared>) -> Self {
        let mut instances = Self::default();
        for declaration in declared {
            let key = InstanceKey::of::<T>(declaration.declares());
            if let Some(frame) = declaration.expressed_in() {
                instances.expressed_in.insert(key.clone(), frame.clone());
            }
            instances.keys.push(key);
        }
        instances
    }

    /// Each instance's key, in declaration order.
    pub fn keys(&self) -> &[InstanceKey] {
        &self.keys
    }

    /// The frame `instance`'s values are expressed in, when it was declared
    /// with one.
    pub fn expressed_in(&self, instance: &InstanceKey) -> Option<&FrameId> {
        self.expressed_in.get(instance)
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
        type Declared = u32;

        fn instance(&self) -> &u32 {
            &self.id
        }

        fn declared_frame(&self) -> Option<&FrameId> {
            None
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

    #[test]
    fn a_bare_declaration_names_no_frame() {
        let declared = DeclaredInstances::of::<Track>([3, 1]);
        let three = InstanceKey::of::<Track>(&3);

        assert_eq!(
            declared.keys(),
            [three.clone(), InstanceKey::of::<Track>(&1)]
        );
        assert_eq!(declared.expressed_in(&three), None);
    }
}
