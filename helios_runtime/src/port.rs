use crate::prelude::Stamped;

use std::{
    any::{Any, TypeId},
    collections::HashMap,
    sync::Arc,
};

use arc_swap::ArcSwap;
use atomic_float::AtomicF64;

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
    pub instance: &'static str,
}

impl ChannelKey {
    pub fn of<T: 'static>() -> Self {
        Self {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            instance: "",
        }
    }

    pub fn named<T: 'static>(instance: &'static str) -> Self {
        Self {
            type_id: TypeId::of::<T>(),
            type_name: std::any::type_name::<T>(),
            instance,
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

#[allow(unused)]
pub struct PortBus {
    slots: HashMap<ChannelKey, ArcSwap<Option<Arc<dyn Any + Send + Sync>>>>,
    signal_keys: Vec<ChannelKey>,
    tick_now: AtomicF64,
}

#[allow(unused)]
impl PortBus {
    pub fn write<T: Any + Send + Sync>(&self, channel: ChannelKey, stamped: Stamped<T>) {
        todo!()
    }

    pub fn read<T: Any + Send + Sync>(&self, channel: ChannelKey) -> Option<Arc<Stamped<T>>> {
        todo!()
    }

    pub fn read_fresh<T: Any + Send + Sync>(
        &self,
        channel: ChannelKey,
        max_age_secs: f64,
    ) -> Option<Arc<Stamped<T>>> {
        todo!()
    }

    pub fn clear_signals(&self) {
        todo!()
    }

    pub(crate) fn set_tick_time(&self, now: f64) {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    struct Foo;
    struct Bar;

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
}
