//! `Sum` — an additive *fold* combinator: reads several same-typed input
//! channels and publishes their sum on one output.
//!
//! The dual of [`Selector`](super::selector::Selector). Where the selector
//! *chooses* one input and republishes it untouched, `Sum` *combines* every
//! present input into a value that existed at no upstream — so it is generic over
//! `T: Add` and stamps its output fresh (this tick, this node) rather than
//! forwarding a source envelope. The control stack folds a feedback and a
//! feedforward `DriveForce` into one commanded force with it, but the node names
//! no roles and does no control math: it sums whatever channels the caller wires.
//!
//! ## Required vs optional is a presence gate
//!
//! Inputs split two ways, and the split matters only at the *gate*:
//!
//! - `required` — every one must be present or the node publishes nothing. A sum
//!   missing a term is silently wrong, worse than no command (which a downstream
//!   reads as last-known-good). Required inputs are also the *only* inputs the
//!   topological sort orders on, so they are read fresh this tick.
//! - `optional` — folded in when present, skipped when absent. Not ordered, so an
//!   optional contributor is read last-known-good (possibly a tick old): fine for
//!   a slowly varying feedforward term, not for a feedback correction.
//!
//! Past the gate the distinction is spent — the node holds a flat set of
//! contributing `Stamped`s and derives two aggregates from it: the summed value
//! and the worst contributing [`Health`].
//!
//! ## A combinator, like `Selector`
//!
//! Runtime-native (wraps no `helios_core` trait), generic over the payload `T`,
//! no input builder, one monomorphization per `T`. All channels are
//! `InternalChannel`: the summed contributions and the output are brain-internal
//! by construction.

use std::{marker::PhantomData, ops::Add, sync::Arc};

use crate::{
    pipeline::descriptor::AlgorithmNodePortDescriptor,
    port::{ChannelError, InternalChannel},
    ChannelKey, Health, PipelineNode, PortDescriptor, Stamped,
};

pub(crate) struct Sum<T: Send + Sync + Clone + Add<Output = T> + 'static> {
    name: Arc<str>,
    descriptor: PortDescriptor,
    required: Vec<ChannelKey>,
    optional: Vec<ChannelKey>,
    output: ChannelKey,
    _marker: PhantomData<fn() -> T>,
}

impl<T: Send + Sync + Clone + Add<Output = T> + 'static> Sum<T> {
    /// Build a `Sum`.
    ///
    /// `required` inputs must all be present at run time for the node to publish;
    /// `optional` inputs are folded in when present. `output` is the channel the
    /// sum is published on. All are `InternalChannel`; each is stored as a
    /// `ChannelKey` for bus access and mirrored into the descriptor — `required`
    /// as required inputs, `optional` as optional inputs.
    pub(crate) fn new(
        name: impl Into<Arc<str>>,
        required: Vec<InternalChannel>,
        optional: Vec<InternalChannel>,
        output: InternalChannel,
    ) -> Self {
        let mut base_descriptor = AlgorithmNodePortDescriptor::new();

        for req in &required {
            base_descriptor = base_descriptor.input_internal(req.clone());
        }

        for opt in &optional {
            base_descriptor = base_descriptor.optional_internal(opt.clone());
        }

        let descriptor = base_descriptor.output_internal(output.clone()).build();

        Self {
            name: name.into(),
            descriptor,
            required: required.into_iter().map(Into::into).collect(),
            optional: optional.into_iter().map(Into::into).collect(),
            output: output.into(),
            _marker: PhantomData,
        }
    }
}

impl<T: Send + Sync + Clone + Add<Output = T> + 'static> PipelineNode for Sum<T> {
    fn name(&self) -> &str {
        self.name.as_ref()
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    /// Read every present input, publish their sum on `output`.
    ///
    /// Returns without publishing if any `required` input is absent. Otherwise the
    /// present required and optional contributions are summed, and the result goes
    /// out in a fresh `Stamped` carrying *this* tick's time, this node as producer,
    /// and the worst contributing `Health` — a synthesized value, not a forwarded
    /// one. Contributions are read last-known-good, not freshness-gated: a
    /// stale-but-present input still contributes and surfaces only through its own
    /// `Health`.
    fn execute(
        &self,
        bus: &crate::port::PortBus,
        _runtime: &dyn crate::AgentRuntime,
        tick: crate::TickContext,
    ) {
        let Some(required) = self
            .required
            .iter()
            .map(|ch| bus.read::<T>(ch.clone()))
            .collect::<Option<Vec<_>>>()
        else {
            return;
        };

        let optional = self
            .optional
            .iter()
            .filter_map(|ch| bus.read::<T>(ch.clone()))
            .collect::<Vec<_>>();

        let Some(total) = required
            .iter()
            .chain(optional.iter())
            .map(|s| s.value.clone())
            .reduce(|a, b| a + b)
        else {
            return;
        };

        let worst = required
            .iter()
            .chain(optional.iter())
            .map(|s| s.health.clone())
            .fold(Health::Ok, |a, b| a.worse_of(b));

        let stamped = Stamped {
            value: total,
            timestamp: tick.now,
            health: worst,
            producer: tick.node_id,
        };

        if let Err(ChannelError::UnknownChannel) = bus.write(self.output.clone(), stamped) {
            tracing::warn!(channel = %self.output, "sum output channel is not wired into the DAG");
        }
    }
}

#[cfg(test)]
mod tests {
    //! Fold / gate / stamp tests for `Sum`, on a trivial addable `Cmd(i32)` to
    //! prove the node is domain-agnostic: it sums whatever channels the caller
    //! wires, naming no control roles. The three control wirings the node exists
    //! to serve — feedback-only `required=[fb]`, feedforward-only `required=[ff]`,
    //! and combined `required=[fb], optional=[ff]` — are all exercised here as
    //! plain single-required and required-plus-optional shapes.

    use super::*;
    use helios_core::frames::transforms::{Convention, ErasedTransform};
    use helios_core::frames::FrameId;

    use crate::port::PortBus;
    use crate::{AgentRuntime, NodeId, TickContext};

    use helios_core::data::primitives::MonotonicTime;

    use nalgebra::Isometry3;

    #[derive(Clone, Debug, PartialEq)]
    struct Cmd(i32);

    impl Add for Cmd {
        type Output = Cmd;
        fn add(self, rhs: Cmd) -> Cmd {
            Cmd(self.0 + rhs.0)
        }
    }

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

    fn stamped(value: i32, timestamp: f64, health: Health, producer: NodeId) -> Stamped<Cmd> {
        Stamped {
            value: Cmd(value),
            timestamp: MonotonicTime(timestamp),
            health,
            producer,
        }
    }

    fn tick(now: f64, node_id: NodeId) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt: 0.1,
            node_id,
        }
    }

    fn bus_for(node: &Sum<Cmd>) -> PortBus {
        PortBus::new(std::slice::from_ref(node.port_descriptor()))
    }

    /// The canonical control wiring: one required (feedback), one optional
    /// (feedforward). Returns the node plus the fb / ff / output keys.
    fn fb_ff_sum() -> (Sum<Cmd>, ChannelKey, ChannelKey, ChannelKey) {
        let fb = InternalChannel::named::<Cmd>("fb");
        let ff = InternalChannel::named::<Cmd>("ff");
        let out = InternalChannel::named::<Cmd>("sum");
        let node = Sum::new("drive_sum", vec![fb.clone()], vec![ff.clone()], out.clone());
        (node, fb.into(), ff.into(), out.into())
    }

    #[test]
    fn sums_required_and_optional_contributions() {
        // FB 10 + FF 5 → 15: both present, both fold in.
        let (node, fb, ff, out) = fb_ff_sum();
        let bus = bus_for(&node);
        bus.write(fb, stamped(10, 1.0, Health::Ok, 1)).unwrap();
        bus.write(ff, stamped(5, 1.0, Health::Ok, 2)).unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        assert_eq!(bus.read::<Cmd>(out).unwrap().value, Cmd(15));
    }

    #[test]
    fn optional_absent_sums_required_alone() {
        // The combined wiring with a silent feedforward: FF never arrives, so the
        // output is the feedback term alone — no phantom zero, no bail.
        let (node, fb, _ff, out) = fb_ff_sum();
        let bus = bus_for(&node);
        bus.write(fb, stamped(10, 1.0, Health::Ok, 1)).unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        assert_eq!(bus.read::<Cmd>(out).unwrap().value, Cmd(10));
    }

    #[test]
    fn required_absent_publishes_nothing() {
        // Only the optional input is present; the required one never arrives. A sum
        // missing a required term is silently wrong, so the node publishes nothing
        // and the downstream holds its last-known-good.
        let (node, _fb, ff, out) = fb_ff_sum();
        let bus = bus_for(&node);
        bus.write(ff, stamped(5, 1.0, Health::Ok, 2)).unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        assert!(bus.read::<Cmd>(out).is_none());
    }

    #[test]
    fn all_required_inputs_are_summed() {
        // Requiredness is n-ary: two required inputs both contribute.
        let a = InternalChannel::named::<Cmd>("a");
        let b = InternalChannel::named::<Cmd>("b");
        let out = InternalChannel::named::<Cmd>("sum");
        let node = Sum::new("pair", vec![a.clone(), b.clone()], vec![], out.clone());
        let bus = bus_for(&node);
        bus.write(a.into(), stamped(3, 1.0, Health::Ok, 1)).unwrap();
        bus.write(b.into(), stamped(4, 1.0, Health::Ok, 2)).unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        assert_eq!(bus.read::<Cmd>(out.into()).unwrap().value, Cmd(7));
    }

    #[test]
    fn a_single_required_input_passes_through() {
        // The FB-only and FF-only validation wirings: required=[x], optional=[].
        // One contributor reduces to itself — an open-loop characterization run.
        let only = InternalChannel::named::<Cmd>("only");
        let out = InternalChannel::named::<Cmd>("sum");
        let node = Sum::new("solo", vec![only.clone()], vec![], out.clone());
        let bus = bus_for(&node);
        bus.write(only.into(), stamped(8, 1.0, Health::Ok, 1))
            .unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        assert_eq!(bus.read::<Cmd>(out.into()).unwrap().value, Cmd(8));
    }

    #[test]
    fn output_health_is_the_worst_contributor() {
        // A degraded feedforward drags the summed command's health down with it,
        // reason preserved, even though the feedback term is Ok.
        let (node, fb, ff, out) = fb_ff_sum();
        let bus = bus_for(&node);
        bus.write(fb, stamped(10, 1.0, Health::Ok, 1)).unwrap();
        bus.write(
            ff,
            stamped(
                5,
                1.0,
                Health::Degraded {
                    reason: "ff fault".into(),
                },
                2,
            ),
        )
        .unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        let out = bus.read::<Cmd>(out).unwrap();
        let Health::Degraded { reason } = &out.health else {
            panic!("expected Degraded, got {:?}", out.health);
        };
        assert_eq!(reason, "ff fault");
    }

    #[test]
    fn output_is_stamped_fresh_not_forwarded() {
        // The dual of Selector's republish-preserves-source-stamp: a sum is a new
        // value born this tick, so it carries neither input's timestamp (1.0) nor
        // either producer (1, 2) — it carries now=2.0 and this node=7.
        let (node, fb, ff, out) = fb_ff_sum();
        let bus = bus_for(&node);
        bus.write(fb, stamped(10, 1.0, Health::Ok, 1)).unwrap();
        bus.write(ff, stamped(5, 1.0, Health::Ok, 2)).unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        let out = bus.read::<Cmd>(out).unwrap();
        assert!((out.timestamp.0 - 2.0).abs() < 1e-9);
        assert_eq!(out.producer, 7);
    }

    #[test]
    fn descriptor_splits_required_and_optional() {
        let (node, fb, ff, out) = fb_ff_sum();
        let d = node.port_descriptor();
        assert_eq!(d.required_inputs, vec![fb]);
        assert_eq!(d.optional_inputs, vec![ff]);
        assert_eq!(d.outputs, vec![out]);
        assert!(d.rate.is_none());
    }
}
