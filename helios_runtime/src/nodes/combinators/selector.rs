//! `Selector` — a select-one *mux* node: forwards one of several same-typed
//! input channels to a single output, unchanged.
//!
//! The mechanism is domain-agnostic. It names no roles and does no math: it
//! reads its typed input channels, picks one by `SelectorPolicy`, and
//! republishes that `Stamped<T>` verbatim on its output. The control stack wires
//! it to prefer a teleop command over the autonomy command and forward the
//! winner as the command the downstream reads — but the same node selects any
//! redundant same-type stream. The caller supplies the channels and the
//! vocabulary; the node owns neither.
//!
//! A *combinator*: runtime-native (wraps no `helios_core` trait), generic over
//! the payload `T`, with no input builder. It reads its inputs as typed channels
//! directly and pays the monomorphization tax — one instantiation per `T`.
//!
//! ## Two input tiers
//!
//! - `preferred` — freshness-gated sources in priority order; the first that is
//!   fresh wins. Declared as optional inputs (a source such as teleop may be
//!   silent this tick).
//! - `base` — the unconditional fallback, used when no preferred source is
//!   fresh. It is the sole **required** input, so the topological sort is always
//!   satisfied.
//!
//! ## Why every channel is `InternalChannel`
//!
//! The output is brain-produced by definition, and the inputs it selects among
//! are brain-internal (a controller output, a host-injected intent). Reference
//! truth (`OracleChannel`) is deliberately not accepted: the kind fence in
//! `AlgorithmNodePortDescriptor` has no oracle-input method, so a selector
//! choosing between an estimate and ground truth would be a mock-licensed node
//! built on `MockNodePortDescriptor`, not this one. Selecting among redundant
//! `SensorChannel` streams likewise needs a different constructor — this node's
//! inputs are `Internal` by construction.

use crate::{
    pipeline::descriptor::AlgorithmNodePortDescriptor,
    port::{ChannelError, InternalChannel},
    ChannelKey, PipelineNode, PortDescriptor,
};

use std::{marker::PhantomData, sync::Arc};

/// A select-one mux over same-typed channels. Construct via `Self::new`.
///
/// Stateless: selection is a pure per-tick read, so `execute` needs no interior
/// mutability. Generic over the payload `T`; `T: Clone` is required to
/// republish the winning `Stamped<T>` unchanged.
pub(crate) struct Selector<T: Send + Sync + Clone + 'static> {
    name: Arc<str>,
    descriptor: PortDescriptor,
    preferred: Vec<ChannelKey>,
    base: ChannelKey,
    output: ChannelKey,
    policy: SelectorPolicy,
    _marker: PhantomData<fn() -> T>,
}

impl<T: Send + Sync + Clone + 'static> Selector<T> {
    /// Build a selector.
    ///
    /// `preferred` are the freshness-gated inputs in priority order; `base` is
    /// the unconditional fallback and the sole required input; `output` is the
    /// channel the winner is republished on. All are `InternalChannel`; they are
    /// stored as `ChannelKey` for bus access and mirrored into the descriptor —
    /// `base` as the required input, `preferred` as optional inputs.
    pub(crate) fn new(
        name: impl Into<Arc<str>>,
        preferred: Vec<InternalChannel>,
        base: InternalChannel,
        output: InternalChannel,
        policy: SelectorPolicy,
    ) -> Self {
        let mut base_descriptor = AlgorithmNodePortDescriptor::new().input_internal(base.clone());

        for pref in &preferred {
            base_descriptor = base_descriptor.optional_internal(pref.clone());
        }

        let descriptor = base_descriptor.output_internal(output.clone()).build();

        Self {
            name: name.into(),
            descriptor,
            preferred: preferred.into_iter().map(Into::into).collect(),
            base: base.into(),
            output: output.into(),
            policy,
            _marker: PhantomData,
        }
    }
}

impl<T: Send + Sync + Clone + 'static> PipelineNode for Selector<T> {
    fn name(&self) -> &str {
        self.name.as_ref()
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    /// Select one input and republish it on `output`, unchanged.
    ///
    /// Under `FreshnessOverride`, the first `preferred` source that is fresh
    /// wins; otherwise the latest `base` value is forwarded. When nothing is
    /// available (cold start), nothing is published. The winning `Stamped` keeps
    /// its original timestamp, producer, and health, so a downstream freshness
    /// check reflects the *source's* age, not this node's tick.
    fn execute(
        &self,
        bus: &crate::port::PortBus,
        _runtime: &dyn crate::AgentRuntime,
        _tick: crate::TickContext,
    ) {
        let winner = match self.policy {
            SelectorPolicy::FreshnessOverride { max_age } => self
                .preferred
                .iter()
                .find_map(|ch| bus.read_fresh::<T>(ch.clone(), max_age))
                .or_else(|| bus.read::<T>(self.base.clone())),
        };

        let Some(winner) = winner else {
            return;
        };

        let forwarded = (*winner).clone();

        if let Err(ChannelError::UnknownChannel) = bus.write(self.output.clone(), forwarded) {
            tracing::warn!(channel = %self.output, "selector output channel is not wired into the DAG");
        }
    }
}

/// How a `Selector` chooses among its inputs.
///
/// A small enum rather than a trait, because there is one policy. Promote it to
/// a strategy trait when a second one (e.g. selection driven by an explicit mode
/// channel) earns its keep.
pub(crate) enum SelectorPolicy {
    /// A preferred source wins while it is *fresh*; otherwise the base source is
    /// forwarded. `max_age` is the staleness threshold in seconds, measured
    /// against the bus tick clock (see `PortBus::read_fresh`).
    FreshnessOverride { max_age: f64 },
}

#[cfg(test)]
mod tests {
    //! Wiring / selection tests for `Selector`. These use plain named channels,
    //! not the control vocabulary, to prove the node is domain-agnostic: it
    //! routes on whatever channels the caller supplies. Payloads are trivial
    //! `Cmd` structs — `Clone + PartialEq + Debug` is all the node and the
    //! assertions need.

    use super::*;
    use helios_core::frames::transforms::{Convention, ErasedTransform};
    use helios_core::frames::FrameId;

    use crate::port::PortBus;
    use crate::{AgentRuntime, Health, NodeId, Stamped, TickContext};

    use helios_core::data::primitives::MonotonicTime;

    use nalgebra::Isometry3;

    #[derive(Clone, Debug, PartialEq)]
    struct Cmd(u32);

    // A second payload type, used only to show the node monomorphizes over `T`.
    #[derive(Clone, Debug, PartialEq)]
    struct OtherCmd(u32);

    struct MockRuntime;

    impl AgentRuntime for MockRuntime {
        fn get_transform(
            &self,
            _: FrameId,
            _: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            Some(ErasedTransform::from_parts(
                Isometry3::identity(),
                Convention::Flu,
                Convention::Flu,
            ))
        }
        fn now(&self) -> MonotonicTime {
            MonotonicTime(0.0)
        }
    }

    fn stamped(value: u32, timestamp: f64, producer: NodeId) -> Stamped<Cmd> {
        Stamped {
            value: Cmd(value),
            timestamp: MonotonicTime(timestamp),
            health: Health::Ok,
            producer,
        }
    }

    fn tick_at(now: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt: 0.1,
            node_id: 99,
        }
    }

    /// A `Cmd` selector with one preferred channel, plus the three channel keys
    /// for writing inputs and reading the output.
    fn make_selector(max_age: f64) -> (Selector<Cmd>, ChannelKey, ChannelKey, ChannelKey) {
        let preferred = InternalChannel::named::<Cmd>("preferred");
        let base = InternalChannel::named::<Cmd>("base");
        let output = InternalChannel::named::<Cmd>("command");
        let node = Selector::new(
            "sel",
            vec![preferred.clone()],
            base.clone(),
            output.clone(),
            SelectorPolicy::FreshnessOverride { max_age },
        );
        (node, preferred.into(), base.into(), output.into())
    }

    fn bus_for(node: &Selector<Cmd>) -> PortBus {
        PortBus::new(std::slice::from_ref(node.port_descriptor()))
    }

    #[test]
    fn fresh_preferred_wins_over_base() {
        let (node, preferred, base, output) = make_selector(3.0);
        let bus = bus_for(&node);
        bus.set_tick_time(10.0);
        bus.write(base, stamped(1, 9.0, 1)).unwrap();
        bus.write(preferred, stamped(2, 9.5, 2)).unwrap();

        node.execute(&bus, &MockRuntime, tick_at(10.0));

        assert_eq!(bus.read::<Cmd>(output).unwrap().value, Cmd(2));
    }

    #[test]
    fn stale_preferred_falls_back_to_base() {
        let (node, preferred, base, output) = make_selector(3.0);
        let bus = bus_for(&node);
        bus.set_tick_time(10.0);
        // preferred is 5s old (> max_age 3) → stale; base is always eligible.
        bus.write(preferred, stamped(2, 5.0, 2)).unwrap();
        bus.write(base, stamped(1, 9.0, 1)).unwrap();

        node.execute(&bus, &MockRuntime, tick_at(10.0));

        assert_eq!(bus.read::<Cmd>(output).unwrap().value, Cmd(1));
    }

    #[test]
    fn absent_preferred_uses_base() {
        let (node, _preferred, base, output) = make_selector(3.0);
        let bus = bus_for(&node);
        bus.set_tick_time(10.0);
        bus.write(base, stamped(1, 9.5, 1)).unwrap();

        node.execute(&bus, &MockRuntime, tick_at(10.0));

        assert_eq!(bus.read::<Cmd>(output).unwrap().value, Cmd(1));
    }

    #[test]
    fn nothing_published_when_all_sources_absent() {
        let (node, _preferred, _base, output) = make_selector(3.0);
        let bus = bus_for(&node);
        bus.set_tick_time(10.0);

        node.execute(&bus, &MockRuntime, tick_at(10.0));

        assert!(bus.read::<Cmd>(output).is_none());
    }

    #[test]
    fn republish_preserves_source_stamp() {
        let (node, preferred, _base, output) = make_selector(3.0);
        let bus = bus_for(&node);
        bus.set_tick_time(10.0);
        bus.write(preferred, stamped(2, 9.5, 7)).unwrap();

        // The tick carries now=10.0, node_id=99 — neither may leak into output.
        node.execute(&bus, &MockRuntime, tick_at(10.0));

        let out = bus.read::<Cmd>(output).unwrap();
        assert_eq!(out.value, Cmd(2));
        assert!((out.timestamp.0 - 9.5).abs() < 1e-9);
        assert_eq!(out.producer, 7);
    }

    #[test]
    fn descriptor_has_base_required_and_preferred_optional() {
        let (node, preferred, base, output) = make_selector(3.0);
        let d = node.port_descriptor();
        assert_eq!(d.required_inputs, vec![base]);
        assert_eq!(d.optional_inputs, vec![preferred]);
        assert_eq!(d.outputs, vec![output]);
        assert!(d.rate.is_none());
    }

    #[test]
    fn node_is_generic_over_payload_type() {
        // The same node type, monomorphized over `OtherCmd`, routes on that
        // type's channels — proving the DAG channel follows `T`, not a fixed
        // payload.
        let base = InternalChannel::named::<OtherCmd>("base");
        let output = InternalChannel::named::<OtherCmd>("command");
        let node = Selector::<OtherCmd>::new(
            "sel_other",
            vec![InternalChannel::named::<OtherCmd>("preferred")],
            base.clone(),
            output.clone(),
            SelectorPolicy::FreshnessOverride { max_age: 3.0 },
        );
        let bus = PortBus::new(std::slice::from_ref(node.port_descriptor()));
        bus.set_tick_time(10.0);
        bus.write(
            base.into(),
            Stamped {
                value: OtherCmd(5),
                timestamp: MonotonicTime(9.9),
                health: Health::Ok,
                producer: 1,
            },
        )
        .unwrap();

        node.execute(&bus, &MockRuntime, tick_at(10.0));

        let out_key: ChannelKey = output.into();
        assert_eq!(bus.read::<OtherCmd>(out_key).unwrap().value, OtherCmd(5));
    }
}
