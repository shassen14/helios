//! `Merge` — a disjoint-union combinator: reads several partial
//! [`ActuatorCommand`]s and publishes their concatenation as one command.
//!
//! The third combinator shape, alongside [`Sum`](super::sum::Sum) and
//! [`Selector`](super::selector::Selector). Where `Sum` *folds* same-typed inputs
//! whose values collapse (`a + b`) and `Selector` *chooses* one input untouched,
//! `Merge` *concatenates* inputs whose setpoints coexist: each contributing
//! command owns a disjoint set of [`ActuatorId`]s, and the merge is their union —
//! the whole terminal command reassembled from the per-actuator-group partials a
//! decoupled control stack emits (a drive leg and a steer leg, each an allocator
//! writing only its own actuators).
//!
//! Unlike `Sum` it is **monomorphic on `ActuatorCommand`**, not generic: its
//! operation is a keyed union, which needs the [`ActuatorId`] key and has no
//! standard-library trait to ride the way `Sum` rides `Add`. `ActuatorCommand` is
//! also the pipeline's one universal terminal, so "merge partial terminals" is
//! inherently about this single type.
//!
//! ## All inputs required; a collision degrades
//!
//! Every input is required — a merged command missing an actuator group would
//! silently drop that group to its fail-safe, worse than the downstream holding
//! last-known-good — so the node publishes nothing until all inputs are present.
//! The disjointness the decoupled wiring guarantees is checked defensively here:
//! two inputs naming the same actuator is a wiring bug, but a runtime path must
//! not panic, so the merge keeps the first sighting, drops the command's
//! [`Health`] to `Degraded`, and warns. The static guarantee that legs own
//! disjoint actuators lives in the assembler.
//!
//! ## A combinator, like `Sum` and `Selector`
//!
//! Runtime-native (wraps no `helios_core` trait), no input builder. All channels
//! are `InternalChannel`: the partial contributions and the merged command are
//! brain-internal by construction.

use std::{collections::HashSet, sync::Arc};

use helios_core::control::actuators::{ActuatorCommand, ActuatorId};

use crate::{
    pipeline::descriptor::AlgorithmNodePortDescriptor,
    port::{ChannelError, InternalChannel},
    ChannelKey, Health, PipelineNode, PortDescriptor, Stamped,
};

pub(crate) struct Merge {
    name: Arc<str>,
    descriptor: PortDescriptor,
    inputs: Vec<ChannelKey>,
    output: ChannelKey,
}

impl Merge {
    pub(crate) fn new(
        name: impl Into<Arc<str>>,
        inputs: Vec<InternalChannel>,
        output: InternalChannel,
    ) -> Self {
        let mut base_descriptor = AlgorithmNodePortDescriptor::new();

        for input in &inputs {
            base_descriptor = base_descriptor.input_internal(input.clone());
        }

        let descriptor = base_descriptor.output_internal(output.clone()).build();

        Self {
            name: name.into(),
            descriptor,
            inputs: inputs.into_iter().map(Into::into).collect(),
            output: output.into(),
        }
    }
}

impl PipelineNode for Merge {
    fn name(&self) -> &str {
        self.name.as_ref()
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    /// Read every input, publish the concatenation of their setpoints on `output`.
    ///
    /// Returns without publishing if any input is absent — every input is
    /// required, since a command missing an actuator group would drop it to a
    /// fail-safe. Otherwise the present commands' setpoints are concatenated into
    /// one fresh [`ActuatorCommand`] carrying *this* tick's time, this node as
    /// producer, and the worst contributing [`Health`]. Inputs are read
    /// last-known-good, not freshness-gated: a stale-but-present contribution
    /// still merges and surfaces only through its own `Health`.
    ///
    /// Two inputs naming the same [`ActuatorId`] is a wiring bug the assembler's
    /// disjointness check should preclude; if one reaches here the merge keeps the
    /// first sighting, drops the output to `Degraded`, and warns rather than
    /// panicking.
    fn execute(
        &self,
        bus: &crate::port::PortBus,
        _runtime: &dyn crate::AgentRuntime,
        tick: crate::TickContext,
    ) {
        let Some(inputs) = self
            .inputs
            .iter()
            .map(|ch| bus.read::<ActuatorCommand>(ch.clone()))
            .collect::<Option<Vec<_>>>()
        else {
            return;
        };

        let mut setpoints = Vec::new();
        let mut seen: HashSet<ActuatorId> = HashSet::new();
        let mut collided = false;

        for contribution in &inputs {
            for sp in contribution.value.setpoints() {
                if seen.insert(sp.actuator().clone()) {
                    setpoints.push(sp.clone());
                } else {
                    collided = true;
                    tracing::warn!(
                        actuator = ?sp.actuator(),
                        "merge got the same actuator from two inputs; keeping the first"
                    );
                }
            }
        }

        let health = inputs
            .iter()
            .map(|s| s.health.clone())
            .fold(Health::Ok, |a, b| a.worse_of(b));

        let health = if collided {
            health.worse_of(Health::Degraded {
                reason: "merge actuator-id collision".into(),
            })
        } else {
            health
        };

        let stamped = Stamped {
            value: ActuatorCommand::new(setpoints),
            timestamp: tick.now,
            health,
            producer: tick.node_id,
        };

        if let Err(ChannelError::UnknownChannel) = bus.write(self.output.clone(), stamped) {
            tracing::warn!(channel = %self.output, "merge output channel is not wired into the DAG");
        }
    }
}

#[cfg(test)]
mod tests {
    //! Concatenate / gate / collision / stamp tests for `Merge`, on real
    //! `ActuatorCommand`s — the node is monomorphic, so there is no stand-in
    //! payload. The wiring the node exists to serve, a drive leg and a steer leg
    //! each owning a disjoint actuator, is the `merges_disjoint_...` case.

    use super::*;

    use helios_core::control::actuators::{ActuatorSetpoint, SetpointValue};
    use helios_core::data::primitives::MonotonicTime;
    use helios_core::frames::transforms::{Convention, ErasedTransform};
    use helios_core::frames::FrameId;

    use crate::port::PortBus;
    use crate::{AgentRuntime, NodeId, TickContext};

    use nalgebra::Isometry3;

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

    // One actuator's setpoint, terse.
    fn sp(id: &str, value: SetpointValue) -> ActuatorSetpoint {
        ActuatorSetpoint::new(ActuatorId::new(id), value)
    }

    // A partial command wrapped in a bus envelope.
    fn stamped(
        setpoints: Vec<ActuatorSetpoint>,
        timestamp: f64,
        health: Health,
        producer: NodeId,
    ) -> Stamped<ActuatorCommand> {
        Stamped {
            value: ActuatorCommand::new(setpoints),
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

    fn bus_for(node: &Merge) -> PortBus {
        PortBus::new(std::slice::from_ref(node.port_descriptor()))
    }

    /// The canonical decoupled wiring: a drive leg and a steer leg merged into one
    /// terminal command. Returns the node plus the drive / steer / output keys.
    fn drive_steer_merge() -> (Merge, ChannelKey, ChannelKey, ChannelKey) {
        let drive = InternalChannel::named::<ActuatorCommand>("drive_leg");
        let steer = InternalChannel::named::<ActuatorCommand>("steer_leg");
        let out = InternalChannel::named::<ActuatorCommand>("actuators");
        let node = Merge::new(
            "actuator_merge",
            vec![drive.clone(), steer.clone()],
            out.clone(),
        );
        (node, drive.into(), steer.into(), out.into())
    }

    #[test]
    fn merges_disjoint_actuator_groups() {
        // A drive Velocity and a steer Position on separate legs concatenate into
        // one command carrying both setpoints, in input order, health Ok.
        let (node, drive, steer, out) = drive_steer_merge();
        let bus = bus_for(&node);
        bus.write(
            drive,
            stamped(vec![sp("drive", SetpointValue::Velocity(3.0))], 1.0, Health::Ok, 1),
        )
        .unwrap();
        bus.write(
            steer,
            stamped(vec![sp("steer", SetpointValue::Position(0.2))], 1.0, Health::Ok, 2),
        )
        .unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        let merged = bus.read::<ActuatorCommand>(out).unwrap();
        assert_eq!(
            merged.value,
            ActuatorCommand::new(vec![
                sp("drive", SetpointValue::Velocity(3.0)),
                sp("steer", SetpointValue::Position(0.2)),
            ])
        );
        assert!(matches!(merged.health, Health::Ok));
    }

    #[test]
    fn a_missing_input_publishes_nothing() {
        // Only the drive leg has fired; the steer leg never arrived. A command
        // missing an actuator group would drop it to fail-safe, so the node
        // publishes nothing and the downstream holds its last-known-good.
        let (node, drive, _steer, out) = drive_steer_merge();
        let bus = bus_for(&node);
        bus.write(
            drive,
            stamped(vec![sp("drive", SetpointValue::Velocity(3.0))], 1.0, Health::Ok, 1),
        )
        .unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        assert!(bus.read::<ActuatorCommand>(out).is_none());
    }

    #[test]
    fn colliding_actuator_id_degrades_and_keeps_first() {
        // Both legs name `drive` — a wiring bug. The merge keeps the first sighting
        // (Velocity 3.0, not the 9.0 from the second leg), drops health to
        // Degraded, and does not panic.
        let (node, drive, steer, out) = drive_steer_merge();
        let bus = bus_for(&node);
        bus.write(
            drive,
            stamped(vec![sp("drive", SetpointValue::Velocity(3.0))], 1.0, Health::Ok, 1),
        )
        .unwrap();
        bus.write(
            steer,
            stamped(vec![sp("drive", SetpointValue::Velocity(9.0))], 1.0, Health::Ok, 2),
        )
        .unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        let merged = bus.read::<ActuatorCommand>(out).unwrap();
        assert_eq!(merged.value.setpoints().len(), 1);
        assert_eq!(
            merged.value.setpoints()[0].value(),
            &SetpointValue::Velocity(3.0)
        );
        assert!(matches!(merged.health, Health::Degraded { .. }));
    }

    #[test]
    fn output_is_stamped_fresh() {
        // The merged command is born this tick: it carries neither input's
        // timestamp (1.0) nor either producer (1, 2), but now=2.0 and this node=7.
        let (node, drive, steer, out) = drive_steer_merge();
        let bus = bus_for(&node);
        bus.write(
            drive,
            stamped(vec![sp("drive", SetpointValue::Velocity(3.0))], 1.0, Health::Ok, 1),
        )
        .unwrap();
        bus.write(
            steer,
            stamped(vec![sp("steer", SetpointValue::Position(0.2))], 1.0, Health::Ok, 2),
        )
        .unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        let merged = bus.read::<ActuatorCommand>(out).unwrap();
        assert!((merged.timestamp.0 - 2.0).abs() < 1e-9);
        assert_eq!(merged.producer, 7);
    }

    #[test]
    fn output_health_is_the_worst_contributor() {
        // A degraded steer leg drags the merged command's health down with it,
        // reason preserved, though the drive leg is Ok and no collision occurred.
        let (node, drive, steer, out) = drive_steer_merge();
        let bus = bus_for(&node);
        bus.write(
            drive,
            stamped(vec![sp("drive", SetpointValue::Velocity(3.0))], 1.0, Health::Ok, 1),
        )
        .unwrap();
        bus.write(
            steer,
            stamped(
                vec![sp("steer", SetpointValue::Position(0.2))],
                1.0,
                Health::Degraded {
                    reason: "steer fault".into(),
                },
                2,
            ),
        )
        .unwrap();

        node.execute(&bus, &MockRuntime, tick(2.0, 7));

        let merged = bus.read::<ActuatorCommand>(out).unwrap();
        let Health::Degraded { reason } = &merged.health else {
            panic!("expected Degraded, got {:?}", merged.health);
        };
        assert_eq!(reason, "steer fault");
    }

    #[test]
    fn descriptor_lists_all_inputs_required() {
        // All inputs are required (none optional), the output is listed, and the
        // combinator is rate-free.
        let (node, drive, steer, out) = drive_steer_merge();
        let d = node.port_descriptor();
        assert_eq!(d.required_inputs, vec![drive, steer]);
        assert!(d.optional_inputs.is_empty());
        assert_eq!(d.outputs, vec![out]);
        assert!(d.rate.is_none());
    }
}
