//! Typed bus channels and the blackboard they live on.
//!
//! # Channel kinds — the partition
//!
//! Every bus channel carries a **kind tag** that says who is allowed to
//! produce it and who is allowed to declare it as input:
//!
//! | Kind | Producer | Algorithm-node input? | Mock-node input? |
//! |---|---|:-:|:-:|
//! | [`SensorChannel`] | Body (driver / sim plugin) | ✅ | ✅ |
//! | [`InternalChannel`] | Algorithm or mock node | ✅ | ✅ |
//! | [`OracleChannel`] | Body, when truth is known | ❌ | ✅ |
//! | [`HealthChannel`] | Body driver layer | ❌ (today) | ❌ (today) |
//!
//! Storage in [`PortBus`] is uniform — one slot per [`ChannelKey`]. The
//! partition is at the **declaration surface**: see the descriptor builders
//! in [`crate::pipeline::descriptor`]. The compiler refuses to construct an
//! algorithm node's descriptor that names an [`OracleChannel`] as input.
//!
//! # Naming convention
//!
//! Within a kind, the `instance` string is just a disambiguator. Across
//! kinds, prefix conventions help humans read the DAG dump:
//!
//! - `OracleChannel` instances start with `oracle/` (enforced by
//!   `debug_assert!`).
//! - `HealthChannel` instances start with `health/` (enforced by
//!   `debug_assert!`).
//! - `SensorChannel` and `InternalChannel` are unconstrained — the type
//!   plus role disambiguator is enough.

use crate::prelude::Stamped;

use std::{
    any::{Any, TypeId},
    collections::HashMap,
    sync::{atomic::Ordering, Arc},
};

use arc_swap::ArcSwap;
use atomic_float::AtomicF64;

/// Error returned when a bus operation references a channel that was not
/// declared in any node's [`PortDescriptor`] at build time.
#[derive(Debug)]
pub enum ChannelError {
    UnknownChannel,
}

/// Discriminator returned by [`ChannelKey::kind`].
#[derive(Clone, Copy, Eq, PartialEq, Hash, Debug)]
pub enum ChannelKind {
    Sensor,
    Internal,
    Oracle,
    Health,
}

impl ChannelKind {
    pub fn as_str(self) -> &'static str {
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
    pub fn kind(&self) -> ChannelKind {
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

/// Declares what a pipeline node reads from and writes to the bus.
///
/// `required_inputs` channels must have a declared producer in the graph for
/// `PipelineBuilder::build()` to succeed. This does NOT guarantee a value is
/// present at runtime — cold-start, sensor dropout, and rate-gated upstream
/// nodes mean every consumer must handle `None`. The standard pattern is an
/// early-return.
///
/// `optional_inputs` channels are consumed if present; no build-time check.
///
/// No two nodes may declare the same `outputs` channel — enforced at build time.
///
/// Construct via [`crate::pipeline::descriptor::AlgorithmNodePortDescriptor`]
/// or [`crate::pipeline::descriptor::MockNodePortDescriptor`] rather than
/// building this struct directly — those builders are the kind fence.
#[derive(Debug)]
pub struct PortDescriptor {
    /// Channels that must have a producer in the graph for the build to succeed.
    pub required_inputs: Vec<ChannelKey>,

    /// Channels the node uses if a value is present. No build-time check.
    pub optional_inputs: Vec<ChannelKey>,

    /// Channels this node writes when it executes.
    pub outputs: Vec<ChannelKey>,

    /// Execution rate in Hz. `None` means every tick.
    pub rate: Option<f64>,
}

/// Typed, lock-free in-memory blackboard for intra-pipeline data exchange.
///
/// All slots are pre-populated at construction from the union of every node's
/// declared inputs and outputs. Reads and writes never block — each slot uses
/// an [`ArcSwap`] for atomic pointer swap with concurrent read access.
///
/// All slots use **last-known-good** semantics — a write replaces the current
/// value; subsequent reads return the most recent write until something else
/// overwrites it. Consumers that need to react only to fresh data must
/// dedupe on [`Stamped::timestamp`] themselves (see [`PortBus::read_fresh`]
/// for the simple max-age helper, or track a per-consumer last-seen
/// timestamp for exact one-shot semantics).
pub struct PortBus {
    slots: HashMap<ChannelKey, ArcSwap<Option<Arc<dyn Any + Send + Sync>>>>,
    tick_now: AtomicF64,
}

impl PortBus {
    /// Constructs a [`PortBus`] pre-populated with one empty slot per unique
    /// [`ChannelKey`] found across all `descriptors`.
    pub fn new<'a>(descriptors: impl IntoIterator<Item = &'a PortDescriptor>) -> Self {
        let mut slots = HashMap::new();

        for descriptor in descriptors {
            for key in descriptor
                .required_inputs
                .iter()
                .chain(descriptor.optional_inputs.iter())
                .chain(descriptor.outputs.iter())
            {
                slots
                    .entry(key.clone())
                    .or_insert_with(|| ArcSwap::new(Arc::new(None)));
            }
        }

        Self {
            slots,
            tick_now: AtomicF64::new(0.0),
        }
    }
}

#[allow(unused)]
impl PortBus {
    pub fn write<T: Any + Send + Sync>(
        &self,
        channel: ChannelKey,
        stamped: Stamped<T>,
    ) -> Result<(), ChannelError> {
        let slot = self
            .slots
            .get(&channel)
            .ok_or(ChannelError::UnknownChannel)?;
        slot.store(Arc::new(Some(Arc::new(stamped))));
        Ok(())
    }

    pub fn read<T: Any + Send + Sync>(&self, channel: ChannelKey) -> Option<Arc<Stamped<T>>> {
        let guard = self.slots.get(&channel)?.load();
        let any_arc = guard.as_ref().as_ref()?;

        Arc::clone(any_arc).downcast::<Stamped<T>>().ok()
    }

    /// Returns the first non-empty slot whose value downcasts to `Stamped<T>`,
    /// ignoring the channel kind and instance name.
    ///
    /// Iteration order is unspecified — use this only when there is exactly
    /// one channel of type `T` in the graph (e.g. a single planner's `Path`)
    /// or when "any one" is acceptable (e.g. debug visualization).
    pub fn read_any<T: Any + Send + Sync>(&self) -> Option<Arc<Stamped<T>>> {
        for slot in self.slots.values() {
            let guard = slot.load();
            let Some(any_arc) = guard.as_ref().as_ref() else {
                continue;
            };
            if let Ok(stamped) = Arc::clone(any_arc).downcast::<Stamped<T>>() {
                return Some(stamped);
            }
        }
        None
    }

    pub fn read_fresh<T: Any + Send + Sync>(
        &self,
        channel: ChannelKey,
        max_age_secs: f64,
    ) -> Option<Arc<Stamped<T>>> {
        let stamped = self.read::<T>(channel)?;
        let now = self.tick_now.load(Ordering::Relaxed);

        if now - stamped.timestamp.0 <= max_age_secs {
            Some(stamped)
        } else {
            None
        }
    }

    pub(crate) fn set_tick_time(&self, now: f64) {
        self.tick_now.store(now, Ordering::Relaxed);
    }

    /// Debug-only: enumerate every declared slot and whether it currently
    /// holds a value. Iteration order is unspecified (HashMap order). Use
    /// from `crate::diagnostics` or tests — not from per-tick code paths.
    pub fn slot_presence(&self) -> Vec<(ChannelKey, bool)> {
        self.slots
            .iter()
            .map(|(key, slot)| (key.clone(), slot.load().as_ref().is_some()))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::{Health, Stamped};
    use helios_core::data::primitives::MonotonicTime;

    struct Foo;
    struct Bar;

    fn make_stamped<T>(value: T, timestamp_secs: f64) -> Stamped<T> {
        Stamped {
            value,
            timestamp: MonotonicTime(timestamp_secs),
            health: Health::Ok,
            producer: 0,
        }
    }

    fn bus_with_outputs(outputs: Vec<ChannelKey>) -> PortBus {
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs,
            rate: None,
        };
        PortBus::new(&[descriptor])
    }

    fn ikey<T: 'static>() -> ChannelKey {
        InternalChannel::of::<T>().into()
    }

    fn ikey_named<T: 'static>(instance: &'static str) -> ChannelKey {
        InternalChannel::named::<T>(instance).into()
    }

    // --- ChannelKey tests ---

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

    // --- PortBus::new tests ---

    #[test]
    fn new_populates_slots_from_all_descriptor_sections() {
        let req = ikey::<u32>();
        let opt = ikey_named::<u32>("opt");
        let out = ikey_named::<u32>("out");
        let descriptor = PortDescriptor {
            required_inputs: vec![req.clone()],
            optional_inputs: vec![opt.clone()],
            outputs: vec![out.clone()],
            rate: None,
        };
        let bus = PortBus::new(&[descriptor]);
        assert!(bus.write(req, make_stamped(1u32, 0.0)).is_ok());
        assert!(bus.write(opt, make_stamped(2u32, 0.0)).is_ok());
        assert!(bus.write(out, make_stamped(3u32, 0.0)).is_ok());
    }

    #[test]
    fn new_deduplicates_shared_keys_across_descriptors() {
        let shared = ikey::<u32>();
        let d1 = PortDescriptor {
            required_inputs: vec![shared.clone()],
            optional_inputs: vec![],
            outputs: vec![],
            rate: None,
        };
        let d2 = PortDescriptor {
            required_inputs: vec![shared.clone()],
            optional_inputs: vec![],
            outputs: vec![],
            rate: None,
        };
        let bus = PortBus::new(&[d1, d2]);
        assert_eq!(bus.slots.len(), 1);
    }

    // --- PortBus read/write tests ---

    #[test]
    fn write_and_read_roundtrip() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(42u32, 1.0)).unwrap();
        assert_eq!(bus.read::<u32>(key).unwrap().value, 42);
    }

    #[test]
    fn read_empty_slot_returns_none() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        assert!(bus.read::<u32>(key).is_none());
    }

    #[test]
    fn read_unknown_channel_returns_none() {
        let bus = PortBus::new(&[]);
        assert!(bus.read::<u32>(ikey::<u32>()).is_none());
    }

    #[test]
    fn write_unknown_channel_returns_error() {
        let bus = PortBus::new(&[]);
        let result = bus.write(ikey::<u32>(), make_stamped(1u32, 0.0));
        assert!(matches!(result, Err(ChannelError::UnknownChannel)));
    }

    #[test]
    fn write_overwrites_previous_value() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();
        bus.write(key.clone(), make_stamped(2u32, 1.0)).unwrap();
        assert_eq!(bus.read::<u32>(key).unwrap().value, 2);
    }

    // --- read_fresh tests ---

    #[test]
    fn read_fresh_returns_none_when_stale() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.set_tick_time(10.0);
        bus.write(key.clone(), make_stamped(1u32, 5.0)).unwrap();
        assert!(bus.read_fresh::<u32>(key, 3.0).is_none());
    }

    #[test]
    fn read_fresh_returns_value_when_current() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.set_tick_time(10.0);
        bus.write(key.clone(), make_stamped(7u32, 9.0)).unwrap();
        assert_eq!(bus.read_fresh::<u32>(key, 3.0).unwrap().value, 7);
    }

    #[test]
    fn read_fresh_returns_none_for_empty_slot() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.set_tick_time(10.0);
        assert!(bus.read_fresh::<u32>(key, 5.0).is_none());
    }
}
