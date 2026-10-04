//! The [`PortBus`] blackboard: one last-known-good slot per channel.
//!
//! [`PortBus`] is a flat, last-known-good blackboard: one slot per
//! [`ChannelKey`](crate::port::ChannelKey), each holding the most recent
//! [`Stamped`](crate::Stamped) value written to it. A read returns that value
//! until it is overwritten — there is no per-tick clear and no queue. Slots are
//! allocated from node [`PortDescriptor`]s at build time, so a write to a
//! channel no node declared is a
//! [`ChannelError::UnknownChannel`](crate::port::ChannelError).
//!
//! Every slot also carries a [`SlotVersion`] the bus bumps on each write, so a
//! consumer can tell a new write from an unchanged value without trusting the
//! producer's timestamp.
//!
//! [`ErasedStamped`] is the type-agnostic view of a slot for tooling
//! (diagnostics, inspector) that holds a [`ChannelKey`](crate::port::ChannelKey)
//! but not the compile-time payload type.

use std::{
    any::{Any, TypeId},
    collections::HashMap,
    sync::{atomic::Ordering, Arc},
};

use arc_swap::ArcSwap;
use atomic_float::AtomicF64;
use helios_core::prelude::MonotonicTime;

use crate::{
    port::{
        channel::{ChannelError, ChannelKey},
        descriptor::PortDescriptor,
    },
    Stamped,
};

/// Type-agnostic view of a [`Stamped<T>`] value on the bus.
///
/// The bus stores values erased so slots are uniform, but pulling `T` back
/// out of a bare `dyn Any` requires naming `T` again — which a runtime
/// consumer (e.g. the test harness's assertion evaluator) cannot do, since
/// it only learns the type as a `TypeId` carried inside the [`ChannelKey`].
/// The fix is to capture the payload projection *at write time*, where `T`
/// is statically known, into this trait's vtable. A later, type-agnostic
/// reader then calls [`payload`](Self::payload) / [`payload_type`](Self::payload_type)
/// without ever naming `T`.
pub trait ErasedStamped: Any + Send + Sync {
    /// The wrapped value, **not** the `Stamped` envelope. This is the `&dyn
    /// Any` an extractor downcasts on; keeping it the bare payload means the
    /// extractor table stays keyed on the payload type, not `Stamped<T>`.
    fn payload(&self) -> &dyn Any;

    /// `TypeId` of the payload `T` — matches the `ChannelKey`'s `type_id()`,
    /// so a consumer can pick the right extractor before downcasting.
    fn payload_type_id(&self) -> TypeId;

    fn timestamp(&self) -> MonotonicTime;

    /// Escape hatch to a plain `dyn Any` so the typed `read`/`read_fresh` paths
    /// can `.downcast::<Stamped<T>>()` — std only downcasts `dyn Any`, never a
    /// custom trait object.
    fn into_any(self: Arc<Self>) -> Arc<dyn Any + Send + Sync>;
}

impl<T: Any + Send + Sync> ErasedStamped for Stamped<T> {
    fn payload(&self) -> &dyn Any {
        &self.value
    }

    fn payload_type_id(&self) -> TypeId {
        self.value.type_id()
    }

    fn timestamp(&self) -> MonotonicTime {
        self.timestamp
    }

    fn into_any(self: Arc<Self>) -> Arc<dyn Any + Send + Sync> {
        self
    }
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
///
/// Each slot also carries a [`SlotVersion`] the bus bumps on every write, read
/// through [`PortBus::read_versioned`]. The timestamp says when the data was
/// measured; the version says which write this is.
pub struct PortBus {
    slots: HashMap<ChannelKey, ArcSwap<SlotEntry>>,
    tick_now: AtomicF64,
}

impl PortBus {
    /// Constructs a [`PortBus`] pre-populated with one empty slot per unique
    /// [`ChannelKey`] found across all `descriptors`.
    ///
    /// The bus represents intra-graph flow: a slot exists iff some node in
    /// the graph mentions the channel. Channels the body *advertises*
    /// via [`BodyCapabilities`](crate::BodyCapabilities) but which no node
    /// consumes intentionally have no slot — host writes return
    /// [`ChannelError::UnknownChannel`] until a consumer is added. The body
    /// declares what it offers; the bus tracks what the graph uses.
    pub fn new<'a>(descriptors: impl IntoIterator<Item = &'a PortDescriptor>) -> Self {
        let mut slots = HashMap::new();

        for descriptor in descriptors {
            for key in descriptor
                .required_inputs()
                .chain(descriptor.optional_inputs())
                .chain(descriptor.outputs().iter())
            {
                slots
                    .entry(key.clone())
                    .or_insert_with(|| ArcSwap::new(Arc::new(SlotEntry::initial())));
            }
        }

        Self {
            slots,
            tick_now: AtomicF64::new(0.0),
        }
    }
}

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
        // Load-then-store is not one atomic step; it cannot race because the
        // build admits exactly one writer per channel.
        let version = slot.load().version.next();

        slot.store(Arc::new(SlotEntry {
            data: Some(Arc::new(stamped) as Arc<dyn ErasedStamped>),
            version,
        }));

        Ok(())
    }

    pub fn read<T: Any + Send + Sync>(&self, channel: ChannelKey) -> Option<Arc<Stamped<T>>> {
        let guard = self.slots.get(&channel)?.load();
        let any_arc = guard.data.as_ref()?;

        Arc::clone(any_arc).into_any().downcast::<Stamped<T>>().ok()
    }

    /// [`read`](Self::read) plus the slot's [`SlotVersion`], both taken from one
    /// load so the pair always belongs to the same write. Use the version to
    /// tell "a new write landed" apart from "same value as last time", which a
    /// producer-set timestamp cannot (late measurements, repeated stamps,
    /// clock jumps). `None` for an unknown channel, an empty slot, or a wrong `T`.
    pub fn read_versioned<T: Any + Send + Sync>(
        &self,
        channel: ChannelKey,
    ) -> Option<(Arc<Stamped<T>>, SlotVersion)> {
        let guard = self.slots.get(&channel)?.load();
        let any_arc = guard.data.as_ref()?;
        let stamped = Arc::clone(any_arc)
            .into_any()
            .downcast::<Stamped<T>>()
            .ok()?;

        Some((stamped, guard.version))
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

    /// Read a slot's current value without naming its Rust type — the one
    /// primitive a runtime consumer (assertion evaluator, bus inspector) can
    /// call when it only has a [`ChannelKey`] and not a compile-time `T`.
    ///
    /// Returns an owned `Arc`, not a borrow: `load()` hands back a temporary
    /// guard, so cloning the inner `Arc` out is what lets the value outlive
    /// this call. `None` covers both an unknown channel and an empty slot.
    pub fn read_erased(&self, key: &ChannelKey) -> Option<Arc<dyn ErasedStamped>> {
        let guard = self.slots.get(key)?.load();
        let erased = guard.data.as_ref()?;
        Some(Arc::clone(erased))
    }

    pub(crate) fn set_tick_time(&self, now: f64) {
        self.tick_now.store(now, Ordering::Relaxed);
    }

    /// Debug-only: enumerate every declared slot and whether it currently
    /// holds a value. Iteration order is unspecified (HashMap order). Use
    /// from `crate::diagnostics` or tests — not from per-tick code paths.
    pub(crate) fn slot_presence(&self) -> Vec<(ChannelKey, bool)> {
        self.slots
            .iter()
            .map(|(key, slot)| (key.clone(), slot.load().data.is_some()))
            .collect()
    }
}

/// What one slot's [`ArcSwap`] holds: the latest value and the version of the
/// write that put it there. They live in one struct so a single atomic swap
/// replaces both — a reader can never see a new value with an old version.
struct SlotEntry {
    data: Option<Arc<dyn ErasedStamped>>,
    version: SlotVersion,
}

impl SlotEntry {
    /// An unwritten slot: no value, version zero.
    fn initial() -> Self {
        Self {
            data: None,
            version: SlotVersion::initial(),
        }
    }
}

/// Counts the writes to one bus slot: zero while empty, then one more per
/// write. It is a write counter, not a time — see [`Stamped::timestamp`] for
/// when the data was measured.
///
/// Only the bus creates versions, so a producer cannot set one wrong. Compare
/// versions only for equality ("has a new write landed since I last looked?");
/// the type deliberately has no ordering. The counter wraps after 2^64 writes,
/// which equality checks survive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SlotVersion(u64);

impl SlotVersion {
    /// The version of a slot that has never been written.
    pub(crate) fn initial() -> Self {
        Self(0u64)
    }

    /// The version the next write to this slot receives.
    pub(crate) fn next(mut self) -> Self {
        self.0 = self.0.wrapping_add(1u64);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::channel::InternalChannel;
    use crate::prelude::Health;

    fn make_stamped<T>(value: T, timestamp_secs: f64) -> Stamped<T> {
        Stamped {
            value,
            timestamp: MonotonicTime(timestamp_secs),
            health: Health::Ok,
            producer: 0,
        }
    }

    fn bus_with_outputs(outputs: Vec<ChannelKey>) -> PortBus {
        let descriptor = PortDescriptor::new(vec![], vec![], outputs, None);
        PortBus::new(&[descriptor])
    }

    fn ikey<T: 'static>() -> ChannelKey {
        InternalChannel::of::<T>().into()
    }

    fn ikey_named<T: 'static>(instance: &'static str) -> ChannelKey {
        InternalChannel::named::<T>(instance).into()
    }

    // --- PortBus::new tests ---

    #[test]
    fn new_populates_slots_from_all_descriptor_sections() {
        let req = ikey::<u32>();
        let opt = ikey_named::<u32>("opt");
        let out = ikey_named::<u32>("out");
        let descriptor = PortDescriptor::new(
            vec![req.clone()],
            vec![opt.clone()],
            vec![out.clone()],
            None,
        );
        let bus = PortBus::new(&[descriptor]);
        assert!(bus.write(req, make_stamped(1u32, 0.0)).is_ok());
        assert!(bus.write(opt, make_stamped(2u32, 0.0)).is_ok());
        assert!(bus.write(out, make_stamped(3u32, 0.0)).is_ok());
    }

    #[test]
    fn new_deduplicates_shared_keys_across_descriptors() {
        let shared = ikey::<u32>();
        let d1 = PortDescriptor::new(vec![shared.clone()], vec![], vec![], None);
        let d2 = PortDescriptor::new(vec![shared.clone()], vec![], vec![], None);
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

    // --- read_erased / ErasedStamped tests ---

    #[test]
    fn read_erased_payload_downcasts_to_value() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(42u32, 1.0)).unwrap();

        let erased = bus.read_erased(&key).unwrap();
        // Payload is the bare value, not the Stamped envelope.
        let value = erased.payload().downcast_ref::<u32>().unwrap();
        assert_eq!(*value, 42);
    }

    #[test]
    fn read_erased_payload_type_matches_typeid() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();

        let erased = bus.read_erased(&key).unwrap();
        // The payload TypeId is what a consumer keys its extractor on, and it
        // must agree with the channel key's own type_id.
        assert_eq!(erased.payload_type_id(), TypeId::of::<u32>());
        assert_eq!(erased.payload_type_id(), key.type_id());
    }

    #[test]
    fn read_erased_exposes_timestamp() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 7.5)).unwrap();

        let erased = bus.read_erased(&key).unwrap();
        assert_eq!(erased.timestamp(), MonotonicTime(7.5));
    }

    #[test]
    fn read_erased_returns_none_for_empty_slot() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        assert!(bus.read_erased(&key).is_none());
    }

    #[test]
    fn read_erased_returns_none_for_unknown_channel() {
        let bus = PortBus::new(&[]);
        assert!(bus.read_erased(&ikey::<u32>()).is_none());
    }

    #[test]
    fn read_erased_payload_rejects_wrong_type() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();

        let erased = bus.read_erased(&key).unwrap();
        // Downcasting to the wrong payload type fails rather than mis-reads.
        assert!(erased.payload().downcast_ref::<i64>().is_none());
    }

    #[test]
    fn into_any_roundtrips_to_stamped() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(99u32, 2.0)).unwrap();

        let erased = bus.read_erased(&key).unwrap();
        // The escape hatch recovers the full Stamped<T>, the same path the
        // typed read::<T> takes internally.
        let stamped = erased.into_any().downcast::<Stamped<u32>>().unwrap();
        assert_eq!(stamped.value, 99);
        assert_eq!(stamped.timestamp, MonotonicTime(2.0));
    }

    // --- PortBus::read_versioned tests ---

    #[test]
    fn first_write_is_version_one_after_initial() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();

        let (stamped, version) = bus.read_versioned::<u32>(key).unwrap();
        assert_eq!(stamped.value, 1);
        assert_eq!(version, SlotVersion::initial().next());
    }

    #[test]
    fn each_write_advances_version_by_one() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();
        let (_, first) = bus.read_versioned::<u32>(key.clone()).unwrap();

        bus.write(key.clone(), make_stamped(2u32, 0.1)).unwrap();
        let (stamped, second) = bus.read_versioned::<u32>(key).unwrap();
        assert_eq!(stamped.value, 2);
        assert_eq!(second, first.next());
    }

    #[test]
    fn version_advances_when_timestamp_repeats() {
        // The case a timestamp dedupe misses: same stamp, new write.
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 5.0)).unwrap();
        let (_, first) = bus.read_versioned::<u32>(key.clone()).unwrap();

        bus.write(key.clone(), make_stamped(2u32, 5.0)).unwrap();
        let (_, second) = bus.read_versioned::<u32>(key).unwrap();
        assert_ne!(first, second);
    }

    #[test]
    fn version_advances_when_timestamp_goes_backwards() {
        // A late measurement carries an older stamp but is still a new write.
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 5.0)).unwrap();
        let (_, first) = bus.read_versioned::<u32>(key.clone()).unwrap();

        bus.write(key.clone(), make_stamped(2u32, 4.0)).unwrap();
        let (_, second) = bus.read_versioned::<u32>(key).unwrap();
        assert_eq!(second, first.next());
    }

    #[test]
    fn versions_count_independently_per_channel() {
        let a = ikey_named::<u32>("a");
        let b = ikey_named::<u32>("b");
        let bus = bus_with_outputs(vec![a.clone(), b.clone()]);
        bus.write(a.clone(), make_stamped(1u32, 0.0)).unwrap();
        bus.write(a.clone(), make_stamped(2u32, 0.1)).unwrap();
        bus.write(b.clone(), make_stamped(3u32, 0.1)).unwrap();

        let (_, version_a) = bus.read_versioned::<u32>(a).unwrap();
        let (_, version_b) = bus.read_versioned::<u32>(b).unwrap();
        assert_eq!(version_a, SlotVersion::initial().next().next());
        assert_eq!(version_b, SlotVersion::initial().next());
    }

    #[test]
    fn read_versioned_returns_none_for_empty_slot() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        assert!(bus.read_versioned::<u32>(key).is_none());
    }

    #[test]
    fn read_versioned_returns_none_for_unknown_channel() {
        let bus = bus_with_outputs(vec![]);
        assert!(bus.read_versioned::<u32>(ikey::<u32>()).is_none());
    }

    #[test]
    fn read_versioned_returns_none_for_wrong_type() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();
        assert!(bus.read_versioned::<i64>(key).is_none());
    }

    #[test]
    fn read_matches_read_versioned_value() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(7u32, 1.0)).unwrap();

        let plain = bus.read::<u32>(key.clone()).unwrap();
        let (versioned, _) = bus.read_versioned::<u32>(key).unwrap();
        assert!(Arc::ptr_eq(&plain, &versioned));
    }

    #[test]
    fn slot_version_wraps_at_max() {
        assert_eq!(SlotVersion(u64::MAX).next(), SlotVersion::initial());
    }
}
