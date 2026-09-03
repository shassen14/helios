//! Channel identity — kinds, per-kind constructors, and [`ChannelKey`].
//!
//! A channel's identity is `(kind, type, instance name)`. The kind tag says who
//! may produce a channel and who may declare it as input; the type and instance
//! name disambiguate slots within a kind. [`PortBus`](crate::port::PortBus)
//! stores one slot per [`ChannelKey`] regardless of kind — the partition is
//! enforced at the *declaration surface* (the descriptor builders in
//! [`crate::pipeline::descriptor`]), not in storage. The compiler refuses to
//! construct an algorithm node's descriptor that names an [`OracleChannel`] as
//! input.
//!
//! # Channel kinds — the partition
//!
//! | Kind | Producer | Algorithm-node input? | Mock-node input? |
//! |---|---|:-:|:-:|
//! | [`SensorChannel`] | Body (driver / sim plugin) | ✅ | ✅ |
//! | [`InternalChannel`] | Algorithm or mock node | ✅ | ✅ |
//! | [`OracleChannel`] | Body, when truth is known | ❌ | ✅ |
//! | [`HealthChannel`] | Body driver layer | ❌ (today) | ❌ (today) |
//!
//! # Naming convention
//!
//! Within a kind, the `instance` string is just a disambiguator. Across kinds,
//! prefix conventions help humans read the DAG dump:
//!
//! - `OracleChannel` instances start with `oracle/` (enforced by `debug_assert!`).
//! - `HealthChannel` instances start with `health/` (enforced by `debug_assert!`).
//! - `SensorChannel` and `InternalChannel` are unconstrained — the type plus
//!   role disambiguator is enough.

use std::{any::TypeId, sync::Arc};

/// Error returned when a bus operation references a channel that was not
/// declared in any node's [`PortDescriptor`](crate::port::PortDescriptor) at
/// build time.
#[derive(Debug)]
pub enum ChannelError {
    UnknownChannel,
}

/// Discriminator returned by [`ChannelKey::kind`].
#[derive(Clone, Copy, Eq, PartialEq, Hash, Debug)]
pub(crate) enum ChannelKind {
    Sensor,
    Internal,
    Oracle,
    Health,
}

impl ChannelKind {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            ChannelKind::Sensor => "sensor",
            ChannelKind::Internal => "internal",
            ChannelKind::Oracle => "oracle",
            ChannelKind::Health => "health",
        }
    }
}

/// Sensor data published by a body driver (real or simulated). Algorithm
/// nodes read these; mocks may read them too.
#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub struct SensorChannel {
    type_id: TypeId,
    type_name: &'static str,
    instance: Arc<str>,
}

impl SensorChannel {
    pub fn of<T: 'static>() -> Self {
        Self {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            instance: Arc::from(""),
        }
    }

    pub fn named<T: 'static>(instance: impl Into<Arc<str>>) -> Self {
        Self {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            instance: instance.into(),
        }
    }

    pub fn type_id(&self) -> TypeId {
        self.type_id
    }
    pub fn type_name(&self) -> &'static str {
        self.type_name
    }
    pub fn instance(&self) -> &Arc<str> {
        &self.instance
    }
}

/// Brain-internal channel: produced by an algorithm or mock node, consumed
/// by other nodes / viz / tests. The default kind for node-to-node values.
#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub struct InternalChannel {
    type_id: TypeId,
    type_name: &'static str,
    /// Disambiguator for multiple channels of the same type. Empty string =
    /// the unnamed/default channel (see [`ChannelKey::of`]).
    ///
    /// Stored as `Arc<str>` so the name can originate from either a `&'static
    /// str` literal (zero-cost via `From<&str>`) or a runtime-built `String`
    /// from TOML config without leaking. Clones are cheap refcount bumps;
    /// equality and hashing compare string contents.
    instance: Arc<str>,
}

impl InternalChannel {
    pub fn of<T: 'static>() -> Self {
        Self {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            instance: Arc::from(""),
        }
    }

    /// Construct a named channel. Accepts anything `Into<Arc<str>>` —
    /// `&'static str` literals at the call site, owned `String`s from
    /// config loading, or pre-built `Arc<str>` from a name registry.
    pub fn named<T: 'static>(instance: impl Into<Arc<str>>) -> Self {
        Self {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            instance: instance.into(),
        }
    }

    pub fn type_id(&self) -> TypeId {
        self.type_id
    }
    pub fn type_name(&self) -> &'static str {
        self.type_name
    }
    pub fn instance(&self) -> &Arc<str> {
        &self.instance
    }
}

/// Reference truth published by a body that happens to know the answer
/// (e.g. physics ground-truth in sim, RTK on hardware). **Algorithm nodes
/// must not declare these as inputs** — only mocks, viz, recorders, and
/// tests may consume them.
///
/// `instance` must start with `oracle/` (enforced by `debug_assert!`) so
/// the prefix is visible in the bus inspector and DAG dump.
#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub struct OracleChannel {
    type_id: TypeId,
    type_name: &'static str,
    instance: Arc<str>,
}

impl OracleChannel {
    pub fn named<T: 'static>(instance: impl Into<Arc<str>>) -> Self {
        let instance: Arc<str> = instance.into();
        debug_assert!(
            instance.starts_with("oracle/"),
            "OracleChannel instance must start with \"oracle/\", got \"{instance}\""
        );
        Self {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            instance,
        }
    }

    pub fn type_id(&self) -> TypeId {
        self.type_id
    }
    pub fn type_name(&self) -> &'static str {
        self.type_name
    }
    pub fn instance(&self) -> &Arc<str> {
        &self.instance
    }
}

/// Driver / fault status published by a body. Reserved for the future
/// safety-supervisor consumer; no descriptor
/// builder accepts `HealthChannel` as input today.
///
/// `instance` must start with `health/` (enforced by `debug_assert!`).
#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub struct HealthChannel {
    type_id: TypeId,
    type_name: &'static str,
    instance: Arc<str>,
}

impl HealthChannel {
    pub fn named<T: 'static>(instance: impl Into<Arc<str>>) -> Self {
        let instance: Arc<str> = instance.into();
        debug_assert!(
            instance.starts_with("health/"),
            "HealthChannel instance must start with \"health/\", got \"{instance}\""
        );
        Self {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            instance,
        }
    }

    pub fn type_id(&self) -> TypeId {
        self.type_id
    }

    pub fn type_name(&self) -> &'static str {
        self.type_name
    }

    pub fn instance(&self) -> &Arc<str> {
        &self.instance
    }
}

/// Kinded identifier for a single bus slot.
///
/// The kind tag is the type-system fence that keeps algorithm nodes from
/// declaring body-only channels (Oracle, Health) as inputs. Two channel
/// keys are equal when their kind, `TypeId`, and `instance` all match —
/// so `oracle/pose` and `pose` are distinct slots even when their Rust
/// payload type is the same.
#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub enum ChannelKey {
    Sensor(SensorChannel),
    Internal(InternalChannel),
    Oracle(OracleChannel),
    Health(HealthChannel),
}

impl ChannelKey {
    pub(crate) fn kind(&self) -> ChannelKind {
        match self {
            ChannelKey::Sensor(_) => ChannelKind::Sensor,
            ChannelKey::Internal(_) => ChannelKind::Internal,
            ChannelKey::Oracle(_) => ChannelKind::Oracle,
            ChannelKey::Health(_) => ChannelKind::Health,
        }
    }

    pub fn type_id(&self) -> TypeId {
        match self {
            ChannelKey::Sensor(c) => c.type_id,
            ChannelKey::Internal(c) => c.type_id,
            ChannelKey::Oracle(c) => c.type_id,
            ChannelKey::Health(c) => c.type_id,
        }
    }

    pub fn type_name(&self) -> &'static str {
        match self {
            ChannelKey::Sensor(c) => c.type_name,
            ChannelKey::Internal(c) => c.type_name,
            ChannelKey::Oracle(c) => c.type_name,
            ChannelKey::Health(c) => c.type_name,
        }
    }

    pub fn instance(&self) -> &Arc<str> {
        match self {
            ChannelKey::Sensor(c) => &c.instance,
            ChannelKey::Internal(c) => &c.instance,
            ChannelKey::Oracle(c) => &c.instance,
            ChannelKey::Health(c) => &c.instance,
        }
    }
}

impl From<SensorChannel> for ChannelKey {
    fn from(c: SensorChannel) -> Self {
        ChannelKey::Sensor(c)
    }
}
impl From<InternalChannel> for ChannelKey {
    fn from(c: InternalChannel) -> Self {
        ChannelKey::Internal(c)
    }
}
impl From<OracleChannel> for ChannelKey {
    fn from(c: OracleChannel) -> Self {
        ChannelKey::Oracle(c)
    }
}
impl From<HealthChannel> for ChannelKey {
    fn from(c: HealthChannel) -> Self {
        ChannelKey::Health(c)
    }
}

impl std::fmt::Display for ChannelKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let kind = self.kind().as_str();
        let type_name = self.type_name();
        let instance = self.instance();
        if instance.trim().is_empty() {
            write!(f, "[{kind}] {type_name}")
        } else {
            write!(f, "[{kind}] {type_name} @ \"{instance}\"")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    struct Foo;
    struct Bar;

    fn ikey<T: 'static>() -> ChannelKey {
        InternalChannel::of::<T>().into()
    }

    fn ikey_named<T: 'static>(instance: &'static str) -> ChannelKey {
        InternalChannel::named::<T>(instance).into()
    }

    #[test]
    fn of_same_type_produces_equal_keys() {
        assert_eq!(ikey::<Foo>(), ikey::<Foo>());
    }

    #[test]
    fn of_different_types_produces_distinct_keys() {
        assert_ne!(ikey::<Foo>(), ikey::<Bar>());
    }

    #[test]
    fn named_differs_from_default_instance() {
        assert_ne!(ikey::<Foo>(), ikey_named::<Foo>("role"));
    }

    #[test]
    fn named_same_instance_equal() {
        assert_eq!(ikey_named::<Foo>("a"), ikey_named::<Foo>("a"));
    }

    #[test]
    fn named_different_instances_not_equal() {
        assert_ne!(ikey_named::<Foo>("a"), ikey_named::<Foo>("b"));
    }

    #[test]
    fn distinct_kinds_with_same_type_and_instance_are_distinct() {
        // Two channels with identical TypeId and instance string but
        // different kinds occupy distinct slots — the kind tag is part of
        // identity.
        let internal: ChannelKey = InternalChannel::of::<Foo>().into();
        let sensor: ChannelKey = SensorChannel::of::<Foo>().into();
        assert_ne!(internal, sensor);
    }

    #[test]
    fn usable_as_hashmap_key() {
        let mut map = HashMap::new();
        map.insert(ikey::<Foo>(), 42u32);
        assert_eq!(map[&ikey::<Foo>()], 42);
    }

    #[test]
    fn type_name_accessor_contains_type_name() {
        let key = ikey::<Foo>();
        assert!(key.type_name().contains("Foo"));
    }

    #[test]
    fn kind_reports_correct_variant() {
        assert_eq!(SensorChannel::of::<Foo>().type_id(), TypeId::of::<Foo>());
        let s: ChannelKey = SensorChannel::of::<Foo>().into();
        let i: ChannelKey = InternalChannel::of::<Foo>().into();
        let o: ChannelKey = OracleChannel::named::<Foo>("oracle/x").into();
        let h: ChannelKey = HealthChannel::named::<Foo>("health/x").into();
        assert_eq!(s.kind(), ChannelKind::Sensor);
        assert_eq!(i.kind(), ChannelKind::Internal);
        assert_eq!(o.kind(), ChannelKind::Oracle);
        assert_eq!(h.kind(), ChannelKind::Health);
    }
}
