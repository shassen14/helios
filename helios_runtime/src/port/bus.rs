//! The [`PortBus`] blackboard: one last-known-good slot per channel.
//!
//! [`PortBus`] is a flat, last-known-good blackboard: one slot per
//! [`ChannelKey`](crate::port::ChannelKey), each holding the most recent
//! [`Stamped`](crate::Stamped) value written to it. A keyed channel holds one
//! such slot per instance its writers declare, in instance order, written
//! with [`PortBus::write_keyed`] and read per instance. A read returns that value
//! until it is overwritten — there is no per-tick clear and no queue. Slots are
//! allocated from the pipeline's [`ChannelDescription`]s at build time, so a
//! write to a channel without one is a
//! [`ChannelError::UnknownChannel`](crate::port::ChannelError), and a write of
//! a value whose type is not the key's is a
//! [`ChannelError::TypeMismatch`](crate::port::ChannelError).
//!
//! A single-value channel whose writers declare mounts takes
//! [`Mounted`] values only, through [`PortBus::write_mounted`], which refuses
//! a value naming a frame no writer declared.
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
    collections::{BTreeMap, HashMap},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};

use arc_swap::ArcSwap;
use atomic_float::AtomicF64;
use helios_core::prelude::MonotonicTime;

use crate::{
    port::{
        channel::{ChannelError, ChannelKey},
        description::ChannelDescription,
        keyed::{InstanceKey, Keyed},
        mounted::Mounted,
    },
    Stamped,
};

use helios_core::spatial::FrameId;

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

    /// The payload's Rust type name, for people reading it (debug output,
    /// tooling). Not a stable identity: compare [`payload_type_id`](Self::payload_type_id)
    /// instead.
    fn payload_type_name(&self) -> &'static str;

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

    fn payload_type_name(&self) -> &'static str {
        std::any::type_name::<T>()
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
/// All slots are pre-populated at construction, one per channel description
/// the build hands it. Reads and writes never block — each slot uses
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
///
/// A keyed channel holds one such slot per declared instance, so each
/// instance keeps its own last value and version. The single-value reads
/// return `None` for a keyed channel, and warn the first time, since the
/// caller meant [`read_instance`](Self::read_instance) or
/// [`read_instances`](Self::read_instances).
pub struct PortBus {
    slots: HashMap<ChannelKey, Slot>,
    tick_now: AtomicF64,
}

impl PortBus {
    /// Constructs a [`PortBus`] with one empty slot per channel in
    /// `descriptions`. Several descriptions of one channel, its writers'
    /// shares of a keyed channel, make one slot holding every instance they
    /// declare; a single-value channel's slot keeps the mounts they declare.
    ///
    /// Which channels get a slot is the caller's choice: the pipeline build
    /// passes the descriptions of channels some node reads or writes, so a
    /// host write to a channel the body offers but no node uses is
    /// [`ChannelError::UnknownChannel`].
    pub(crate) fn new<'a>(descriptions: impl IntoIterator<Item = &'a ChannelDescription>) -> Self {
        let mut slots: HashMap<ChannelKey, Slot> = HashMap::new();
        for description in descriptions {
            let key = description.key();
            slots
                .entry(key.clone())
                .or_insert_with(|| Slot::empty(key))
                .declare(description);
        }

        Self {
            slots,
            tick_now: AtomicF64::new(0.0),
        }
    }

    /// A bus for testing nodes alone: each descriptor's outputs described as
    /// a node's, with their instances and no mounts, and its inputs as
    /// outside inputs.
    #[cfg(test)]
    pub(crate) fn of_descriptors<'a>(
        descriptors: impl IntoIterator<Item = &'a crate::port::PortDescriptor>,
    ) -> Self {
        const TEST_WRITER: &str = "node_under_test";

        let descriptions: Vec<ChannelDescription> = descriptors
            .into_iter()
            .flat_map(|descriptor| {
                let inputs = descriptor
                    .required_inputs()
                    .chain(descriptor.optional_inputs())
                    .map(|key| ChannelDescription::outside(key.clone()));
                let outputs = descriptor.output_ports().iter().map(|output| {
                    ChannelDescription::node(output.channel().clone(), TEST_WRITER)
                        .with_instances(output.instances().to_vec())
                });
                inputs.chain(outputs).collect::<Vec<_>>()
            })
            .collect();
        Self::new(&descriptions)
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
        let Slot::Single { entry, mounts } = slot else {
            return Err(ChannelError::KeyedChannel);
        };
        if !mounts.is_empty() {
            return Err(ChannelError::MountedChannel);
        }
        check_payload_type::<T>(&channel)?;
        store(entry, stamped);

        Ok(())
    }

    /// Writes `stamped`, whose values each name the frame they were measured
    /// in, to single-value `channel`. Refused whole if any value names a
    /// frame no writer of the channel declared; a channel whose writers
    /// declare no mount takes any frame.
    pub fn write_mounted<T: Mounted>(
        &self,
        channel: ChannelKey,
        stamped: Stamped<T>,
    ) -> Result<(), ChannelError> {
        let slot = self
            .slots
            .get(&channel)
            .ok_or(ChannelError::UnknownChannel)?;
        let Slot::Single { entry, mounts } = slot else {
            return Err(ChannelError::KeyedChannel);
        };
        check_payload_type::<T>(&channel)?;
        check_mounts(mounts, &stamped.value)?;
        store(entry, stamped);

        Ok(())
    }

    /// Writes `stamped` to the instance of keyed `channel` its value names.
    /// Refused for a channel that holds one value, and for an instance no
    /// writer declared.
    pub fn write_keyed<T: Keyed>(
        &self,
        channel: ChannelKey,
        stamped: Stamped<T>,
    ) -> Result<(), ChannelError> {
        let slot = self
            .slots
            .get(&channel)
            .ok_or(ChannelError::UnknownChannel)?;
        let Slot::Keyed { instances, .. } = slot else {
            return Err(ChannelError::NotKeyed);
        };
        check_payload_type::<T>(&channel)?;
        let instance = InstanceKey::of::<T>(stamped.value.instance());
        let Some(entry) = instances.get(&instance) else {
            return Err(ChannelError::UndeclaredInstance { instance });
        };
        store(entry, stamped);

        Ok(())
    }

    pub fn read<T: Any + Send + Sync>(&self, channel: ChannelKey) -> Option<Arc<Stamped<T>>> {
        self.read_versioned(channel).map(|(stamped, _)| stamped)
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
        let (erased, version) = self.read_erased_versioned(&channel)?;
        let stamped = erased.into_any().downcast::<Stamped<T>>().ok()?;

        Some((stamped, version))
    }

    /// The latest value of `instance` of keyed `channel`. `None` for an
    /// unknown channel or instance, an unwritten instance, or a wrong `T`.
    pub fn read_instance<T: Any + Send + Sync>(
        &self,
        channel: ChannelKey,
        instance: &InstanceKey,
    ) -> Option<Arc<Stamped<T>>> {
        let (erased, _) = self.read_erased_instance_versioned(&channel, instance)?;
        erased.into_any().downcast::<Stamped<T>>().ok()
    }

    /// The latest value of every written instance of keyed `channel`, in
    /// instance order. Empty for an unknown or single-value channel, or a
    /// wrong `T`.
    pub fn read_instances<T: Any + Send + Sync>(
        &self,
        channel: ChannelKey,
    ) -> Vec<(InstanceKey, Arc<Stamped<T>>)> {
        let Some(Slot::Keyed { instances, .. }) = self.slots.get(&channel) else {
            return Vec::new();
        };
        instances
            .iter()
            .filter_map(|(instance, entry)| {
                let erased = Arc::clone(entry.load().data.as_ref()?);
                let stamped = erased.into_any().downcast::<Stamped<T>>().ok()?;
                Some((instance.clone(), stamped))
            })
            .collect()
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

    /// A slot's current value, without naming its Rust type, and its
    /// [`SlotVersion`], both taken from one load so the pair always belongs to
    /// the same write. Every single-value read goes through here; the typed
    /// reads downcast what it returns. The pipeline also calls it directly,
    /// comparing versions around a node's run to tell whether that node wrote
    /// a channel it has no compile-time `T` for. `None` for an unknown
    /// channel, a keyed one, or an empty slot. A keyed channel holds no one
    /// value, so reading it this way is the caller's mistake: it warns the
    /// first time, naming the channel, rather than leave the reader running
    /// on nothing.
    pub(crate) fn read_erased_versioned(
        &self,
        key: &ChannelKey,
    ) -> Option<(Arc<dyn ErasedStamped>, SlotVersion)> {
        match self.slots.get(key)? {
            Slot::Single { entry, .. } => load(entry),
            Slot::Keyed { misread_warned, .. } => {
                if !misread_warned.swap(true, Ordering::Relaxed) {
                    tracing::warn!(
                        channel = %key,
                        "a keyed channel was read as one value, which it never \
                         holds; read it per instance"
                    );
                }
                None
            }
        }
    }

    /// [`read_erased_versioned`](Self::read_erased_versioned) for `instance`
    /// of keyed channel `key`. `None` for an unknown channel or instance, a
    /// single-value channel, or an unwritten instance.
    pub(crate) fn read_erased_instance_versioned(
        &self,
        key: &ChannelKey,
        instance: &InstanceKey,
    ) -> Option<(Arc<dyn ErasedStamped>, SlotVersion)> {
        let Slot::Keyed { instances, .. } = self.slots.get(key)? else {
            return None;
        };
        load(instances.get(instance)?)
    }

    pub(crate) fn set_tick_time(&self, now: f64) {
        self.tick_now.store(now, Ordering::Relaxed);
    }

    /// Debug-only: enumerate every declared channel and whether it currently
    /// holds a value, for a keyed channel in any instance. Iteration order is
    /// unspecified (HashMap order). Use from `crate::diagnostics` or tests —
    /// not from per-tick code paths.
    pub(crate) fn slot_presence(&self) -> Vec<(ChannelKey, bool)> {
        self.slots
            .iter()
            .map(|(key, slot)| {
                let present = match slot {
                    Slot::Single { entry, .. } => entry.load().data.is_some(),
                    Slot::Keyed { instances, .. } => {
                        instances.values().any(|entry| entry.load().data.is_some())
                    }
                };
                (key.clone(), present)
            })
            .collect()
    }
}

/// One channel's storage: a single slot, or a keyed channel's slot per
/// instance. The instances are fixed when the bus is built, so writes and
/// reads of one instance never touch another's.
enum Slot {
    Single {
        entry: ArcSwap<SlotEntry>,
        /// The frames the channel's writers declare its values are measured
        /// in. Empty when none declares one, and then writes go unchecked.
        mounts: Vec<FrameId>,
    },
    Keyed {
        instances: BTreeMap<InstanceKey, ArcSwap<SlotEntry>>,
        /// Whether a single-value read of the channel has been warned of.
        misread_warned: AtomicBool,
    },
}

impl Slot {
    /// The empty storage `key` takes: a keyed channel with no instances yet,
    /// or one unwritten slot.
    fn empty(key: &ChannelKey) -> Self {
        if key.is_keyed() {
            Slot::Keyed {
                instances: BTreeMap::new(),
                misread_warned: AtomicBool::new(false),
            }
        } else {
            Slot::Single {
                entry: SlotEntry::empty(),
                mounts: Vec::new(),
            }
        }
    }

    /// Adds what `description` declares: its instances to a keyed channel,
    /// its mount to a single-value one.
    fn declare(&mut self, description: &ChannelDescription) {
        match self {
            Slot::Keyed { instances, .. } => {
                for instance in description.instances() {
                    instances
                        .entry(instance.clone())
                        .or_insert_with(SlotEntry::empty);
                }
            }
            Slot::Single { mounts, .. } => {
                if let Some(mount) = description.mount() {
                    if !mounts.contains(mount) {
                        mounts.push(mount.clone());
                    }
                }
            }
        }
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

    /// An unwritten slot, ready to swap values into.
    fn empty() -> ArcSwap<Self> {
        ArcSwap::new(Arc::new(Self::initial()))
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

/// Refuses a value of another type than `channel`'s payload. Stored, it would
/// land, and every typed read of the key would downcast it to nothing: the
/// reader runs as if the channel were never written.
fn check_payload_type<T: Any>(channel: &ChannelKey) -> Result<(), ChannelError> {
    if channel.type_id() == TypeId::of::<T>() {
        Ok(())
    } else {
        Err(ChannelError::TypeMismatch {
            key_type: channel.type_name(),
            written_type: std::any::type_name::<T>(),
        })
    }
}

/// Refuses `value` if one of its values names a frame outside `declared`,
/// naming the first. Nothing is checked when `declared` is empty.
fn check_mounts<T: Mounted>(declared: &[FrameId], value: &T) -> Result<(), ChannelError> {
    if declared.is_empty() {
        return Ok(());
    }
    match value.mounts().find(|frame| !declared.contains(frame)) {
        Some(written) => Err(ChannelError::UndeclaredMount {
            declared: declared.to_vec(),
            written: written.clone(),
        }),
        None => Ok(()),
    }
}

/// Puts `stamped` in `entry` under the next version.
fn store<T: Any + Send + Sync>(entry: &ArcSwap<SlotEntry>, stamped: Stamped<T>) {
    // Load-then-store is not one atomic step; it cannot race because the
    // build admits exactly one writer per channel, or per instance of a keyed
    // channel.
    let version = entry.load().version.next();

    entry.store(Arc::new(SlotEntry {
        data: Some(Arc::new(stamped) as Arc<dyn ErasedStamped>),
        version,
    }));
}

/// `entry`'s value and version, from one load. `None` while unwritten.
///
/// Returns an owned `Arc`, not a borrow: `load()` hands back a temporary
/// guard, so cloning the inner `Arc` out is what lets the value outlive the
/// call.
fn load(entry: &ArcSwap<SlotEntry>) -> Option<(Arc<dyn ErasedStamped>, SlotVersion)> {
    let guard = entry.load();
    let erased = guard.data.as_ref()?;
    Some((Arc::clone(erased), guard.version))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::channel::ChannelKey;
    use crate::port::descriptor::{PortDescriptor, PortDescriptorBuilder};
    use crate::port::kind::ChannelKind;
    use crate::prelude::Health;

    use helios_core::prelude::{AgentId, SensorReading};

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
        PortBus::of_descriptors(&[descriptor])
    }

    fn ikey<T: 'static>() -> ChannelKey {
        ChannelKey::internal_of::<T>()
    }

    fn ikey_named<T: 'static>(name: &'static str) -> ChannelKey {
        ChannelKey::internal::<T>(name)
    }

    // --- construction tests ---

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
        let bus = PortBus::of_descriptors(&[descriptor]);
        assert!(bus.write(req, make_stamped(1u32, 0.0)).is_ok());
        assert!(bus.write(opt, make_stamped(2u32, 0.0)).is_ok());
        assert!(bus.write(out, make_stamped(3u32, 0.0)).is_ok());
    }

    #[test]
    fn new_deduplicates_shared_keys_across_descriptors() {
        let shared = ikey::<u32>();
        let d1 = PortDescriptor::new(vec![shared.clone()], vec![], vec![], None);
        let d2 = PortDescriptor::new(vec![shared.clone()], vec![], vec![], None);
        let bus = PortBus::of_descriptors(&[d1, d2]);
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
        let bus = PortBus::of_descriptors(&[]);
        assert!(bus.read::<u32>(ikey::<u32>()).is_none());
    }

    #[test]
    fn write_unknown_channel_returns_error() {
        let bus = PortBus::of_descriptors(&[]);
        let result = bus.write(ikey::<u32>(), make_stamped(1u32, 0.0));
        assert!(matches!(result, Err(ChannelError::UnknownChannel)));
    }

    #[test]
    fn write_of_another_type_is_refused_and_leaves_the_slot_empty() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        let result = bus.write(key.clone(), make_stamped(1i64, 0.0));
        assert!(matches!(result, Err(ChannelError::TypeMismatch { .. })));
        assert!(bus.read_erased_versioned(&key).is_none());
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

    // --- ErasedStamped tests ---

    #[test]
    fn erased_payload_downcasts_to_value() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(42u32, 1.0)).unwrap();

        let (erased, _) = bus.read_erased_versioned(&key).unwrap();
        // Payload is the bare value, not the Stamped envelope.
        let value = erased.payload().downcast_ref::<u32>().unwrap();
        assert_eq!(*value, 42);
    }

    #[test]
    fn erased_payload_type_matches_typeid() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();

        let (erased, _) = bus.read_erased_versioned(&key).unwrap();
        // The payload TypeId is what a consumer keys its extractor on, and it
        // must agree with the channel key's own type_id.
        assert_eq!(erased.payload_type_id(), TypeId::of::<u32>());
        assert_eq!(erased.payload_type_id(), key.type_id());
    }

    #[test]
    fn erased_exposes_timestamp() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 7.5)).unwrap();

        let (erased, _) = bus.read_erased_versioned(&key).unwrap();
        assert_eq!(erased.timestamp(), MonotonicTime(7.5));
    }

    #[test]
    fn erased_payload_rejects_wrong_type() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();

        let (erased, _) = bus.read_erased_versioned(&key).unwrap();
        // Downcasting to the wrong payload type fails rather than mis-reads.
        assert!(erased.payload().downcast_ref::<i64>().is_none());
    }

    #[test]
    fn into_any_roundtrips_to_stamped() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(99u32, 2.0)).unwrap();

        let (erased, _) = bus.read_erased_versioned(&key).unwrap();
        // The escape hatch recovers the full Stamped<T>, the same path the
        // typed reads take.
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

    // --- PortBus::read_erased_versioned tests ---

    #[test]
    fn read_erased_versioned_matches_the_typed_reads() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();
        bus.write(key.clone(), make_stamped(2u32, 0.1)).unwrap();

        let (erased, version) = bus.read_erased_versioned(&key).unwrap();
        let (_, typed_version) = bus.read_versioned::<u32>(key.clone()).unwrap();
        let (again, _) = bus.read_erased_versioned(&key).unwrap();
        // The same allocation the slot holds, so a tap shares the value
        // rather than copying it.
        assert!(Arc::ptr_eq(&erased, &again));
        assert_eq!(version, typed_version);
    }

    #[test]
    fn read_erased_versioned_advances_on_each_write() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        bus.write(key.clone(), make_stamped(1u32, 5.0)).unwrap();
        let (_, first) = bus.read_erased_versioned(&key).unwrap();

        // A repeated stamp is still a new write.
        bus.write(key.clone(), make_stamped(2u32, 5.0)).unwrap();
        let (_, second) = bus.read_erased_versioned(&key).unwrap();
        assert_eq!(second, first.next());
    }

    #[test]
    fn read_erased_versioned_returns_none_for_empty_slot() {
        let key = ikey::<u32>();
        let bus = bus_with_outputs(vec![key.clone()]);
        assert!(bus.read_erased_versioned(&key).is_none());
    }

    #[test]
    fn read_erased_versioned_returns_none_for_unknown_channel() {
        let bus = bus_with_outputs(vec![]);
        assert!(bus.read_erased_versioned(&ikey::<u32>()).is_none());
    }

    #[test]
    fn slot_version_wraps_at_max() {
        assert_eq!(SlotVersion(u64::MAX).next(), SlotVersion::initial());
    }

    // --- keyed channels ---

    /// A value about one of several frames, keyed by the frame.
    struct Edge {
        child: &'static str,
        x: f64,
    }

    impl Keyed for Edge {
        type Instance = &'static str;

        fn instance(&self) -> &&'static str {
            &self.child
        }
    }

    fn edges() -> ChannelKey {
        ChannelKey::keyed::<Edge>(ChannelKind::Internal, "edges")
    }

    fn edge(child: &'static str, x: f64) -> Stamped<Edge> {
        make_stamped(Edge { child, x }, x)
    }

    fn instance(child: &'static str) -> InstanceKey {
        InstanceKey::of::<Edge>(&child)
    }

    /// A bus whose one writer declares `children` of the keyed channel.
    fn bus_writing(children: &[&'static str]) -> PortBus {
        let descriptor = PortDescriptorBuilder::algorithm()
            .output_keyed::<Edge>(edges(), children.iter().copied())
            .build();
        PortBus::of_descriptors(&[descriptor])
    }

    /// Two writers' shares of one keyed channel make one slot holding both.
    #[test]
    fn writers_of_one_keyed_channel_share_its_slot() {
        let bus = PortBus::new(&[
            ChannelDescription::node(edges(), "relay").with_instances(vec![instance("base")]),
            ChannelDescription::node(edges(), "localizer").with_instances(vec![instance("odom")]),
        ]);
        assert_eq!(bus.slots.len(), 1);
        bus.write_keyed(edges(), edge("base", 1.0)).unwrap();
        bus.write_keyed(edges(), edge("odom", 2.0)).unwrap();
        assert_eq!(bus.read_instances::<Edge>(edges()).len(), 2);
    }

    #[test]
    fn each_instance_keeps_its_own_last_value() {
        let bus = bus_writing(&["wheel", "base"]);
        bus.write_keyed(edges(), edge("base", 1.0)).unwrap();
        bus.write_keyed(edges(), edge("wheel", 2.0)).unwrap();
        bus.write_keyed(edges(), edge("base", 3.0)).unwrap();

        let base = bus
            .read_instance::<Edge>(edges(), &instance("base"))
            .unwrap();
        let wheel = bus
            .read_instance::<Edge>(edges(), &instance("wheel"))
            .unwrap();
        assert_eq!(base.value.x, 3.0);
        assert_eq!(wheel.value.x, 2.0);
    }

    #[test]
    fn read_instances_lists_written_instances_in_instance_order() {
        let bus = bus_writing(&["wheel", "base", "arm"]);
        bus.write_keyed(edges(), edge("wheel", 1.0)).unwrap();
        bus.write_keyed(edges(), edge("base", 2.0)).unwrap();

        let read = bus.read_instances::<Edge>(edges());
        let spelled: Vec<&str> = read.iter().map(|(key, _)| key.as_str()).collect();
        // Sorted by spelling, and the unwritten `arm` is left out.
        assert_eq!(spelled, ["base", "wheel"]);
        assert_eq!(read[0].1.value.x, 2.0);
    }

    #[test]
    fn versions_count_per_instance() {
        let bus = bus_writing(&["wheel", "base"]);
        bus.write_keyed(edges(), edge("base", 1.0)).unwrap();
        bus.write_keyed(edges(), edge("base", 2.0)).unwrap();
        bus.write_keyed(edges(), edge("wheel", 3.0)).unwrap();

        let (_, base) = bus
            .read_erased_instance_versioned(&edges(), &instance("base"))
            .unwrap();
        let (_, wheel) = bus
            .read_erased_instance_versioned(&edges(), &instance("wheel"))
            .unwrap();
        assert_eq!(base, SlotVersion::initial().next().next());
        assert_eq!(wheel, SlotVersion::initial().next());
    }

    #[test]
    fn a_write_of_an_undeclared_instance_is_refused() {
        let bus = bus_writing(&["base"]);
        let result = bus.write_keyed(edges(), edge("wheel", 1.0));
        assert!(matches!(
            result,
            Err(ChannelError::UndeclaredInstance { instance: refused }) if refused == instance("wheel")
        ));
        assert!(bus.read_instances::<Edge>(edges()).is_empty());
    }

    #[test]
    fn single_value_and_keyed_writes_each_refuse_the_other_shape() {
        let bus = bus_writing(&["base"]);
        assert!(matches!(
            bus.write(edges(), edge("base", 1.0)),
            Err(ChannelError::KeyedChannel)
        ));

        let single = ChannelKey::internal::<Edge>("edge");
        let bus = bus_with_outputs(vec![single.clone()]);
        assert!(matches!(
            bus.write_keyed(single, edge("base", 1.0)),
            Err(ChannelError::NotKeyed)
        ));
    }

    /// A single-value read of a keyed channel returns nothing and marks the
    /// channel warned, so a reader polling it every tick warns once.
    #[test]
    fn single_value_reads_of_a_keyed_channel_return_nothing_and_warn_once() {
        let bus = bus_writing(&["base"]);
        bus.write_keyed(edges(), edge("base", 1.0)).unwrap();
        let warned = || match bus.slots.get(&edges()) {
            Some(Slot::Keyed { misread_warned, .. }) => misread_warned.load(Ordering::Relaxed),
            _ => panic!("edges is keyed"),
        };
        assert!(!warned());

        assert!(bus.read::<Edge>(edges()).is_none());
        assert!(warned());
        assert!(bus.read_erased_versioned(&edges()).is_none());
        assert!(bus
            .read_instance::<Edge>(edges(), &instance("base"))
            .is_some());
    }

    #[test]
    fn a_keyed_channel_is_present_once_any_instance_is_written() {
        let bus = bus_writing(&["wheel", "base"]);
        assert_eq!(bus.slot_presence(), vec![(edges(), false)]);
        bus.write_keyed(edges(), edge("wheel", 1.0)).unwrap();
        assert_eq!(bus.slot_presence(), vec![(edges(), true)]);
    }

    // --- mounted channels ---

    type Batch = Vec<SensorReading<f64>>;

    fn scans() -> ChannelKey {
        ChannelKey::measurement::<Batch>("scans")
    }

    fn frame(leaf: &str) -> FrameId {
        FrameId::agent(AgentId::new("car"), leaf)
    }

    fn batch(leaves: &[&str]) -> Stamped<Batch> {
        let readings = leaves
            .iter()
            .map(|leaf| SensorReading {
                sensor: frame(leaf),
                timestamp: MonotonicTime(0.0),
                data: 0.0,
            })
            .collect();
        make_stamped(readings, 0.0)
    }

    /// A bus whose `scans` channel is written by one node, mounted at
    /// `mount` or at none.
    fn bus_mounted_at(mount: Option<&str>) -> PortBus {
        PortBus::new(&[ChannelDescription::node(scans(), "deproject").with_mount(mount.map(frame))])
    }

    #[test]
    fn a_batch_at_the_declared_mount_is_written() {
        let bus = bus_mounted_at(Some("lidar"));
        bus.write_mounted(scans(), batch(&["lidar", "lidar"]))
            .unwrap();
        assert_eq!(bus.read::<Batch>(scans()).unwrap().value.len(), 2);
    }

    /// One reading at another frame refuses the whole batch, naming the
    /// declared and written frames.
    #[test]
    fn a_batch_naming_an_undeclared_frame_is_refused_whole() {
        let bus = bus_mounted_at(Some("lidar"));
        let result = bus.write_mounted(scans(), batch(&["lidar", "radar"]));
        let Err(error) = result else {
            panic!("a reading at radar must be refused");
        };
        assert!(matches!(
            &error,
            ChannelError::UndeclaredMount { declared, written }
                if declared == &[frame("lidar")] && written == &frame("radar")
        ));
        let message = error.to_string();
        assert!(message.contains("agent.car.radar"), "{message}");
        assert!(message.contains("agent.car.lidar"), "{message}");
        assert!(bus.read::<Batch>(scans()).is_none());
    }

    /// Several writers' mounts are a set: a batch may mix any of them.
    #[test]
    fn a_batch_may_mix_frames_the_channel_declares() {
        let bus = PortBus::new(&[
            ChannelDescription::node(scans(), "front").with_mount(Some(frame("lidar_front"))),
            ChannelDescription::node(scans(), "rear").with_mount(Some(frame("lidar_rear"))),
        ]);
        bus.write_mounted(scans(), batch(&["lidar_front", "lidar_rear"]))
            .unwrap();
    }

    /// A channel whose writers declare no mount takes any frame, and plain
    /// writes too.
    #[test]
    fn an_unmounted_channel_goes_unchecked() {
        let bus = bus_mounted_at(None);
        bus.write_mounted(scans(), batch(&["anything"])).unwrap();
        bus.write(scans(), batch(&["anything"])).unwrap();
    }

    #[test]
    fn a_plain_write_to_a_mounted_channel_is_refused() {
        let bus = bus_mounted_at(Some("lidar"));
        assert!(matches!(
            bus.write(scans(), batch(&["lidar"])),
            Err(ChannelError::MountedChannel)
        ));
        assert!(bus.read::<Batch>(scans()).is_none());
    }

    #[test]
    fn an_empty_batch_passes_the_mount_check() {
        let bus = bus_mounted_at(Some("lidar"));
        bus.write_mounted(scans(), batch(&[])).unwrap();
    }
}
