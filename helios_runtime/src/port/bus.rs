//! The [`PortBus`] blackboard and the [`PortDescriptor`] node-I/O declaration.
//!
//! [`PortBus`] is a flat, last-known-good blackboard: one slot per
//! [`ChannelKey`](crate::port::ChannelKey), each holding the most recent
//! [`Stamped`](crate::Stamped) value written to it. A read returns that value
//! until it is overwritten — there is no per-tick clear and no queue. Slots are
//! allocated from node [`PortDescriptor`]s at build time, so a write to a
//! channel no node declared is a
//! [`ChannelError::UnknownChannel`](crate::port::ChannelError).
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
use helios_core::data::MonotonicTime;

use crate::{
    port::channel::{ChannelError, ChannelKey},
    Stamped,
};

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
pub struct PortBus {
    slots: HashMap<ChannelKey, ArcSwap<Option<Arc<dyn ErasedStamped>>>>,
    tick_now: AtomicF64,
}

impl PortBus {
    /// Constructs a [`PortBus`] pre-populated with one empty slot per unique
    /// [`ChannelKey`] found across all `descriptors`.
    ///
    /// The bus represents intra-graph flow: a slot exists iff some node in
    /// the graph mentions the channel. Channels the host body *advertises*
    /// via [`BodyCapabilities`](crate::BodyCapabilities) but which no node
    /// consumes intentionally have no slot — host writes return
    /// [`ChannelError::UnknownChannel`] until a consumer is added. The body
    /// declares intent; the bus tracks reality.
    ///
    /// The one out-of-graph consumer is `read_control`: a control-consuming
    /// body reads the `command` channel back even when no node produces it, so
    /// the pipeline reserves that slot separately via [`ensure_slot`](Self::ensure_slot).
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
        slot.store(Arc::new(Some(Arc::new(stamped) as Arc<dyn ErasedStamped>)));
        Ok(())
    }

    pub fn read<T: Any + Send + Sync>(&self, channel: ChannelKey) -> Option<Arc<Stamped<T>>> {
        let guard = self.slots.get(&channel)?.load();
        let any_arc = guard.as_ref().as_ref()?;

        Arc::clone(any_arc).into_any().downcast::<Stamped<T>>().ok()
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
        let erased = guard.as_ref().as_ref()?;
        Some(Arc::clone(erased))
    }

    pub(crate) fn ensure_slot(&mut self, key: ChannelKey) {
        self.slots
            .entry(key)
            .or_insert_with(|| ArcSwap::new(Arc::new(None)));
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
            .map(|(key, slot)| (key.clone(), slot.load().as_ref().is_some()))
            .collect()
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
}
