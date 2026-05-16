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

/// Identifies a single bus slot. `type_id` carries the data shape; `instance`
/// carries the role. Default instance is `""`.
///
/// Two channel keys are equal when both `TypeId` and `instance` match — this is
/// what allows two nodes to produce the same Rust type for different roles
/// (e.g. `Path @ "raw"` vs `Path @ "smoothed"`) without a newtype per role.
#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub struct ChannelKey {
    pub type_id: TypeId,
    pub type_name: &'static str,
    /// Disambiguator for multiple channels of the same type. Empty string =
    /// the unnamed/default channel (see [`ChannelKey::of`]).
    ///
    /// Stored as `Arc<str>` so the name can originate from either a `&'static
    /// str` literal (zero-cost via `From<&str>`) or a runtime-built `String`
    /// from TOML config without leaking. Clones are cheap refcount bumps;
    /// equality and hashing compare string contents.
    pub instance: Arc<str>,
}

impl ChannelKey {
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
}

impl std::fmt::Display for ChannelKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.instance.trim().is_empty() {
            write!(f, "{}", self.type_name)
        } else {
            write!(f, "{} @ \"{}\"", self.type_name, self.instance)
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
/// Two slot semantics coexist:
/// - **Signal slots** are cleared at the start of each tick via [`PortBus::clear_signals`].
///   These hold per-tick sensor batches (e.g. `Vec<SensorReading<LinearAcceleration3D>>`)
///   that must not carry over across ticks.
/// - **State slots** use last-known-good semantics and persist until a node overwrites them.
///   A 1 Hz planner's `Path` output stays readable by a 200 Hz controller on every tick
///   between fires.
pub struct PortBus {
    slots: HashMap<ChannelKey, ArcSwap<Option<Arc<dyn Any + Send + Sync>>>>,
    signal_keys: Vec<ChannelKey>,
    tick_now: AtomicF64,
}

impl PortBus {
    /// Constructs a [`PortBus`] pre-populated with one empty slot per unique
    /// [`ChannelKey`] found across all `descriptors`.
    ///
    /// `signal_keys` identifies which slots are cleared each tick by
    /// [`PortBus::clear_signals`]. All other slots use last-known-good semantics.
    /// The caller (typically [`PipelineBuilder`]) is responsible for classifying
    /// signals from the sensor suite configuration.
    pub fn new<'a>(
        descriptors: impl IntoIterator<Item = &'a PortDescriptor>,
        signal_keys: Vec<ChannelKey>,
    ) -> Self {
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
            signal_keys,
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

    pub fn clear_signals(&self) {
        for key in &self.signal_keys {
            if let Some(slot) = self.slots.get(key) {
                slot.store(Arc::new(None));
            }
        }
    }

    pub(crate) fn set_tick_time(&self, now: f64) {
        self.tick_now.store(now, Ordering::Relaxed);
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

    fn bus_with_outputs(outputs: Vec<ChannelKey>, signal_keys: Vec<ChannelKey>) -> PortBus {
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs,
            rate: None,
        };
        PortBus::new(&[descriptor], signal_keys)
    }

    // --- ChannelKey tests ---

    #[test]
    fn of_same_type_produces_equal_keys() {
        assert_eq!(ChannelKey::of::<Foo>(), ChannelKey::of::<Foo>());
    }

    #[test]
    fn of_different_types_produces_distinct_keys() {
        assert_ne!(ChannelKey::of::<Foo>(), ChannelKey::of::<Bar>());
    }

    #[test]
    fn named_differs_from_default_instance() {
        assert_ne!(ChannelKey::of::<Foo>(), ChannelKey::named::<Foo>("role"));
    }

    #[test]
    fn named_same_instance_equal() {
        assert_eq!(ChannelKey::named::<Foo>("a"), ChannelKey::named::<Foo>("a"));
    }

    #[test]
    fn named_different_instances_not_equal() {
        assert_ne!(ChannelKey::named::<Foo>("a"), ChannelKey::named::<Foo>("b"));
    }

    #[test]
    fn usable_as_hashmap_key() {
        let mut map = HashMap::new();
        map.insert(ChannelKey::of::<Foo>(), 42u32);
        assert_eq!(map[&ChannelKey::of::<Foo>()], 42);
    }

    #[test]
    fn type_name_field_contains_type_name() {
        let key = ChannelKey::of::<Foo>();
        assert!(key.type_name.contains("Foo"));
    }

    #[test]
    fn named_type_name_matches_unnamed() {
        let plain = ChannelKey::of::<Foo>();
        let named = ChannelKey::named::<Foo>("role");
        assert_eq!(plain.type_name, named.type_name);
    }

    // --- PortBus::new tests ---

    #[test]
    fn new_populates_slots_from_all_descriptor_sections() {
        let req = ChannelKey::of::<u32>();
        let opt = ChannelKey::named::<u32>("opt");
        let out = ChannelKey::named::<u32>("out");
        let descriptor = PortDescriptor {
            required_inputs: vec![req.clone()],
            optional_inputs: vec![opt.clone()],
            outputs: vec![out.clone()],
            rate: None,
        };
        let bus = PortBus::new(&[descriptor], vec![]);
        assert!(bus.write(req, make_stamped(1u32, 0.0)).is_ok());
        assert!(bus.write(opt, make_stamped(2u32, 0.0)).is_ok());
        assert!(bus.write(out, make_stamped(3u32, 0.0)).is_ok());
    }

    #[test]
    fn new_deduplicates_shared_keys_across_descriptors() {
        let shared = ChannelKey::of::<u32>();
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
        let bus = PortBus::new(&[d1, d2], vec![]);
        assert_eq!(bus.slots.len(), 1);
    }

    // --- PortBus read/write tests ---

    #[test]
    fn write_and_read_roundtrip() {
        let key = ChannelKey::of::<u32>();
        let bus = bus_with_outputs(vec![key.clone()], vec![]);
        bus.write(key.clone(), make_stamped(42u32, 1.0)).unwrap();
        assert_eq!(bus.read::<u32>(key).unwrap().value, 42);
    }

    #[test]
    fn read_empty_slot_returns_none() {
        let key = ChannelKey::of::<u32>();
        let bus = bus_with_outputs(vec![key.clone()], vec![]);
        assert!(bus.read::<u32>(key).is_none());
    }

    #[test]
    fn read_unknown_channel_returns_none() {
        let bus = PortBus::new(&[], vec![]);
        assert!(bus.read::<u32>(ChannelKey::of::<u32>()).is_none());
    }

    #[test]
    fn write_unknown_channel_returns_error() {
        let bus = PortBus::new(&[], vec![]);
        let result = bus.write(ChannelKey::of::<u32>(), make_stamped(1u32, 0.0));
        assert!(matches!(result, Err(ChannelError::UnknownChannel)));
    }

    #[test]
    fn write_overwrites_previous_value() {
        let key = ChannelKey::of::<u32>();
        let bus = bus_with_outputs(vec![key.clone()], vec![]);
        bus.write(key.clone(), make_stamped(1u32, 0.0)).unwrap();
        bus.write(key.clone(), make_stamped(2u32, 1.0)).unwrap();
        assert_eq!(bus.read::<u32>(key).unwrap().value, 2);
    }

    // --- clear_signals tests ---

    #[test]
    fn clear_signals_clears_signal_slot() {
        let key = ChannelKey::of::<u32>();
        let bus = bus_with_outputs(vec![key.clone()], vec![key.clone()]);
        bus.write(key.clone(), make_stamped(99u32, 0.0)).unwrap();
        bus.clear_signals();
        assert!(bus.read::<u32>(key).is_none());
    }

    #[test]
    fn clear_signals_leaves_state_slot_intact() {
        let sig = ChannelKey::of::<u32>();
        let state = ChannelKey::named::<u32>("state");
        let bus = bus_with_outputs(vec![sig.clone(), state.clone()], vec![sig.clone()]);
        bus.write(sig.clone(), make_stamped(1u32, 0.0)).unwrap();
        bus.write(state.clone(), make_stamped(2u32, 0.0)).unwrap();
        bus.clear_signals();
        assert!(bus.read::<u32>(sig).is_none());
        assert_eq!(bus.read::<u32>(state).unwrap().value, 2);
    }

    // --- read_fresh tests ---

    #[test]
    fn read_fresh_returns_none_when_stale() {
        let key = ChannelKey::of::<u32>();
        let bus = bus_with_outputs(vec![key.clone()], vec![]);
        bus.set_tick_time(10.0);
        bus.write(key.clone(), make_stamped(1u32, 5.0)).unwrap();
        assert!(bus.read_fresh::<u32>(key, 3.0).is_none());
    }

    #[test]
    fn read_fresh_returns_value_when_current() {
        let key = ChannelKey::of::<u32>();
        let bus = bus_with_outputs(vec![key.clone()], vec![]);
        bus.set_tick_time(10.0);
        bus.write(key.clone(), make_stamped(7u32, 9.0)).unwrap();
        assert_eq!(bus.read_fresh::<u32>(key, 3.0).unwrap().value, 7);
    }

    #[test]
    fn read_fresh_returns_none_for_empty_slot() {
        let key = ChannelKey::of::<u32>();
        let bus = bus_with_outputs(vec![key.clone()], vec![]);
        bus.set_tick_time(10.0);
        assert!(bus.read_fresh::<u32>(key, 5.0).is_none());
    }
}
