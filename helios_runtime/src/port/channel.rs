//! Channel identity: [`ChannelKey`], and why the bus refuses a write.
//!
//! A channel's identity is `(kind, type, name)`. The [`ChannelKind`] says
//! what the data is; the payload type and the name tell channels of one kind
//! apart. [`PortBus`](crate::port::PortBus) stores one slot per key whatever
//! its kind. Which nodes may read and write each kind is the table in
//! [`kind`](crate::port::kind), checked when the pipeline is built.
//!
//! # Naming
//!
//! The name is the channel's one address on the agent, in watcher paths,
//! recordings and bridges. It is one or more levels joined by `.`
//! ([`is_dotted_name`](helios_core::kernel::identifier::is_dotted_name)), and
//! names exactly one channel per agent; the pipeline build refuses a
//! malformed name and a name shared by two keys. The kind lives in the key,
//! never in the name, so an oracle channel needs no `oracle` prefix.
//!
//! # Keyed channels
//!
//! A channel of a [`Keyed`] payload may be declared
//! [`keyed`](ChannelKey::keyed): the bus then keeps one value per instance
//! the payload names, and writers use
//! [`write_keyed`](crate::port::PortBus::write_keyed). Whether a channel is
//! keyed is part of its key, so a reader and a writer that disagree name two
//! keys under one name, which the build refuses.

use crate::port::keyed::{InstanceKey, Keyed};
use crate::port::kind::ChannelKind;

use std::{any::TypeId, sync::Arc};

/// Identifier for a single bus slot: the channel's kind, payload type and
/// name, and whether it keeps one value per instance.
///
/// Two keys are equal when all four match. The pipeline build still refuses
/// one name used for two keys, so a name differing only in kind or type
/// never reaches the bus.
#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub struct ChannelKey {
    kind: ChannelKind,
    type_id: TypeId,
    type_name: &'static str,
    /// Stored as `Arc<str>` so the name can come from a `&'static str`
    /// literal or a `String` read from config without leaking. Clones are
    /// refcount bumps; equality and hashing compare contents.
    name: Arc<str>,
    keyed: bool,
}

impl ChannelKey {
    /// A `kind` channel carrying `T`, named `name`.
    pub fn new<T: 'static>(kind: ChannelKind, name: impl Into<Arc<str>>) -> Self {
        Self {
            kind,
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            name: name.into(),
            keyed: false,
        }
    }

    /// A `kind` channel carrying `T`, named `name`, that keeps the last
    /// value of each instance `T` names.
    pub fn keyed<T: Keyed>(kind: ChannelKind, name: impl Into<Arc<str>>) -> Self {
        Self {
            keyed: true,
            ..Self::new::<T>(kind, name)
        }
    }

    /// A measurement channel carrying `T`, named `name`. A host
    /// publishing a device's readings takes the name from the agent profile.
    pub fn measurement<T: 'static>(name: impl Into<Arc<str>>) -> Self {
        Self::new::<T>(ChannelKind::Measurement, name)
    }

    /// An internal channel carrying `T`, named `name`.
    pub fn internal<T: 'static>(name: impl Into<Arc<str>>) -> Self {
        Self::new::<T>(ChannelKind::Internal, name)
    }

    /// A reference-truth channel carrying `T`, named `name`.
    pub fn oracle<T: 'static>(name: impl Into<Arc<str>>) -> Self {
        Self::new::<T>(ChannelKind::Oracle, name)
    }

    /// A device-health channel carrying `T`, named `name`.
    pub fn health<T: 'static>(name: impl Into<Arc<str>>) -> Self {
        Self::new::<T>(ChannelKind::Health, name)
    }

    /// The unnamed internal channel carrying `T`, for wiring inside the
    /// pipeline where one channel of the type is all there is.
    pub fn internal_of<T: 'static>() -> Self {
        Self::internal::<T>("")
    }

    pub fn kind(&self) -> ChannelKind {
        self.kind
    }

    pub fn type_id(&self) -> TypeId {
        self.type_id
    }

    pub fn type_name(&self) -> &'static str {
        self.type_name
    }

    pub fn name(&self) -> &Arc<str> {
        &self.name
    }

    /// Whether the channel keeps one value per instance rather than one
    /// value.
    pub fn is_keyed(&self) -> bool {
        self.keyed
    }
}

impl std::fmt::Display for ChannelKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let kind = self.kind.as_str();
        let type_name = self.type_name;
        let name = &self.name;
        if name.trim().is_empty() {
            write!(f, "[{kind}] {type_name}")?;
        } else {
            write!(f, "[{kind}] {type_name} @ \"{name}\"")?;
        }
        if self.keyed {
            write!(f, " (keyed)")?;
        }
        Ok(())
    }
}

/// Why the bus refused a write.
#[derive(Debug)]
pub enum ChannelError {
    /// The channel was not declared in any node's
    /// [`PortDescriptor`](crate::port::PortDescriptor) at build time, so it
    /// has no slot.
    UnknownChannel,
    /// The value written is not of the key's payload type. Stored, it would
    /// read back as nothing for every reader of the key.
    TypeMismatch {
        key_type: &'static str,
        written_type: &'static str,
    },
    /// A single value was written to a keyed channel, which holds one value
    /// per instance; write it with
    /// [`write_keyed`](crate::port::PortBus::write_keyed).
    KeyedChannel,
    /// A keyed write was made to a channel that holds one value.
    NotKeyed,
    /// The value belongs to an instance no writer of the channel declared.
    /// Stored, it would have a writer the build never checked.
    UndeclaredInstance { instance: InstanceKey },
}

impl std::fmt::Display for ChannelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ChannelError::UnknownChannel => write!(f, "no node reads or writes the channel"),
            ChannelError::TypeMismatch {
                key_type,
                written_type,
            } => write!(
                f,
                "the channel carries {key_type} but the value written is {written_type}"
            ),
            ChannelError::KeyedChannel => {
                write!(f, "the channel is keyed, so it takes keyed writes only")
            }
            ChannelError::NotKeyed => {
                write!(
                    f,
                    "the channel holds one value, so it takes no keyed writes"
                )
            }
            ChannelError::UndeclaredInstance { instance } => {
                write!(
                    f,
                    "no writer of the channel declares instance \"{instance}\""
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::collections::HashMap;

    struct Foo;
    struct Bar;

    struct Edge {
        child: u32,
    }

    impl Keyed for Edge {
        type Instance = u32;

        fn instance(&self) -> &u32 {
            &self.child
        }
    }

    #[test]
    fn of_same_type_produces_equal_keys() {
        assert_eq!(
            ChannelKey::internal_of::<Foo>(),
            ChannelKey::internal_of::<Foo>()
        );
    }

    #[test]
    fn of_different_types_produces_distinct_keys() {
        assert_ne!(
            ChannelKey::internal_of::<Foo>(),
            ChannelKey::internal_of::<Bar>()
        );
    }

    #[test]
    fn named_differs_from_unnamed() {
        assert_ne!(
            ChannelKey::internal_of::<Foo>(),
            ChannelKey::internal::<Foo>("role")
        );
    }

    #[test]
    fn named_same_name_equal() {
        assert_eq!(
            ChannelKey::internal::<Foo>("a"),
            ChannelKey::internal::<Foo>("a")
        );
    }

    #[test]
    fn named_different_names_not_equal() {
        assert_ne!(
            ChannelKey::internal::<Foo>("a"),
            ChannelKey::internal::<Foo>("b")
        );
    }

    /// Two channels with one payload type and name but different kinds
    /// are distinct keys: the kind is part of identity.
    #[test]
    fn distinct_kinds_with_same_type_and_name_are_distinct() {
        assert_ne!(
            ChannelKey::internal::<Foo>("pose"),
            ChannelKey::oracle::<Foo>("pose")
        );
        assert_ne!(
            ChannelKey::internal::<Foo>("pose"),
            ChannelKey::measurement::<Foo>("pose")
        );
    }

    #[test]
    fn usable_as_hashmap_key() {
        let mut map = HashMap::new();
        map.insert(ChannelKey::internal_of::<Foo>(), 42u32);
        assert_eq!(map[&ChannelKey::internal_of::<Foo>()], 42);
    }

    #[test]
    fn type_name_accessor_contains_type_name() {
        assert!(ChannelKey::internal_of::<Foo>().type_name().contains("Foo"));
    }

    #[test]
    fn each_constructor_tags_its_kind() {
        assert_eq!(
            ChannelKey::measurement::<Foo>("x").type_id(),
            TypeId::of::<Foo>()
        );
        assert_eq!(
            ChannelKey::measurement::<Foo>("x").kind(),
            ChannelKind::Measurement
        );
        assert_eq!(
            ChannelKey::internal::<Foo>("x").kind(),
            ChannelKind::Internal
        );
        assert_eq!(ChannelKey::oracle::<Foo>("x").kind(), ChannelKind::Oracle);
        assert_eq!(ChannelKey::health::<Foo>("x").kind(), ChannelKind::Health);
    }

    #[test]
    fn display_names_the_kind_type_and_name() {
        let key = ChannelKey::measurement::<Foo>("gps");
        assert_eq!(
            key.to_string(),
            format!("[measurement] {} @ \"gps\"", key.type_name())
        );
    }

    /// A keyed channel and a single-value channel of one type and name are
    /// distinct keys, which the build then refuses as one name for two keys.
    #[test]
    fn keyed_and_single_value_keys_are_distinct() {
        let keyed = ChannelKey::keyed::<Edge>(ChannelKind::Internal, "tf");
        assert!(keyed.is_keyed());
        assert!(!ChannelKey::internal::<Edge>("tf").is_keyed());
        assert_ne!(keyed, ChannelKey::internal::<Edge>("tf"));
        assert_eq!(
            keyed,
            ChannelKey::keyed::<Edge>(ChannelKind::Internal, "tf")
        );
    }

    #[test]
    fn display_marks_a_keyed_channel() {
        let key = ChannelKey::keyed::<Edge>(ChannelKind::Internal, "tf");
        assert_eq!(
            key.to_string(),
            format!("[internal] {} @ \"tf\" (keyed)", key.type_name())
        );
    }
}
