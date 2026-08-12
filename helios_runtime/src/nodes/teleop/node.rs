//! [`TwistTeleopNode`] — the velocity family's operator-intent mapper.
//!
//! Reads a normalised [`TwistIntent`] (dimensionless per-axis deflection the host
//! publishes from a device) and scales each axis by a per-DOF magnitude into a
//! [`BodyTwist`] for the command arbiter. It is the control-input dual of a
//! forward sensor model: the host does the device read it alone can do, this node
//! owns the portable mapping, so the same mapping runs in sim and on hardware.
//!
//! Runtime-native — it wraps no `helios_core` trait and closes no error loop: an
//! input in, a command out, open-loop. The subspace a body drives is data, not
//! type — a car's `sway`/`heave`/`roll`/`pitch` scale is zero, so its output
//! occupies the unicycle subspace with no special-casing here.

use std::sync::Arc;

use helios_core::{
    control::commands::{BodyTwist, TwistIntent},
    frames::quantities::FluVector,
};

use crate::{
    pipeline::descriptor::AlgorithmNodePortDescriptor,
    port::{ChannelError, InternalChannel, PortBus},
    AgentRuntime, ChannelKey, PipelineNode, PortDescriptor, Stamped, TickContext,
};

/// Per-DOF scale from dimensionless intent to a body twist: translation in m/s,
/// rotation in rad/s, applied componentwise. A zero on an axis a body cannot
/// drive keeps that axis out of the output, so the tuning is what selects the
/// subspace.
pub(crate) struct TwistScale {
    pub surge: f64,
    pub sway: f64,
    pub heave: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

/// Scales [`TwistIntent`] on `intent_key` into a [`BodyTwist`] on `output` by a
/// fixed [`TwistScale`]. Stateless; construct via [`Self::new`].
pub(crate) struct TwistTeleopNode {
    name: Arc<str>,
    descriptor: PortDescriptor,
    intent_key: ChannelKey,
    output: ChannelKey,
    scale: TwistScale,
}

impl TwistTeleopNode {
    pub(crate) fn new(
        name: impl Into<Arc<str>>,
        intent: InternalChannel,
        output: InternalChannel,
        scale: TwistScale,
    ) -> Self {
        let descriptor = AlgorithmNodePortDescriptor::new()
            .input_internal(intent.clone())
            .output_internal(output.clone())
            .build();

        Self {
            name: name.into(),
            descriptor,
            intent_key: intent.into(),
            output: output.into(),
            scale,
        }
    }
}

impl PipelineNode for TwistTeleopNode {
    fn name(&self) -> &str {
        self.name.as_ref()
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    /// Scale the latest intent into a twist and publish it.
    ///
    /// The output keeps the *intent's* timestamp, not this tick's, so a
    /// downstream freshness check reflects when the operator last acted: release
    /// the control and the command goes stale, and the arbiter falls back to
    /// autonomy. No intent on the bus — cold start, or a host that publishes only
    /// on active input — publishes nothing, which is deadman-safe. The intent is
    /// re-clamped defensively, so a device that oversteps `[-1, 1]` cannot drive
    /// past the configured magnitude.
    fn execute(&self, bus: &PortBus, _runtime: &dyn AgentRuntime, tick: TickContext) {
        let Some(intent) = bus.read::<TwistIntent>(self.intent_key.clone()) else {
            return;
        };

        let i = intent.value.clamped();
        let s = &self.scale;

        let twist = BodyTwist::new(
            FluVector::new(i.surge * s.surge, i.sway * s.sway, i.heave * s.heave),
            FluVector::new(i.roll * s.roll, i.pitch * s.pitch, i.yaw * s.yaw),
        );

        let stamped = Stamped {
            value: twist,
            timestamp: intent.timestamp,
            health: intent.health.clone(),
            producer: tick.node_id,
        };

        if let Err(ChannelError::UnknownChannel) = bus.write(self.output.clone(), stamped) {
            tracing::warn!(channel = %self.output, "teleop mapper
  output channel is not wired into the DAG");
        }
    }
}

#[cfg(test)]
mod tests {
    //! Wiring / scaling tests for [`TwistTeleopNode`]. The intent contract itself
    //! (`TwistIntent::clamped`, neutral) is tested in `helios_core`; here we check
    //! the node scales onto the right DOF, carries the intent's stamp, and stays
    //! silent when no intent is present.

    use super::*;

    use crate::Health;

    use helios_core::data::primitives::{FrameHandle, MonotonicTime};

    use nalgebra::Isometry3;

    struct MockRuntime;

    impl AgentRuntime for MockRuntime {
        fn get_transform(&self, _: FrameHandle, _: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn world_pose(&self, _: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn now(&self) -> MonotonicTime {
            MonotonicTime(0.0)
        }
    }

    /// A car's tuning: only surge and yaw are active, so whatever the other
    /// intent axes carry, the output can only lie in the unicycle subspace.
    fn car_scale() -> TwistScale {
        TwistScale {
            surge: 4.0,
            sway: 0.0,
            heave: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 1.0,
        }
    }

    fn make_node(scale: TwistScale) -> TwistTeleopNode {
        TwistTeleopNode::new(
            "teleop_mapper",
            InternalChannel::of::<TwistIntent>(),
            InternalChannel::of::<BodyTwist>(),
            scale,
        )
    }

    fn intent_key() -> ChannelKey {
        InternalChannel::of::<TwistIntent>().into()
    }

    fn output_key() -> ChannelKey {
        InternalChannel::of::<BodyTwist>().into()
    }

    fn bus_for(node: &TwistTeleopNode) -> PortBus {
        PortBus::new(std::slice::from_ref(node.port_descriptor()))
    }

    fn write_intent(bus: &PortBus, intent: TwistIntent, timestamp: f64) {
        bus.write(
            intent_key(),
            Stamped {
                value: intent,
                timestamp: MonotonicTime(timestamp),
                health: Health::Ok,
                producer: 1,
            },
        )
        .unwrap();
    }

    fn tick_at(now: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt: 0.1,
            node_id: 7,
        }
    }

    #[test]
    fn scales_each_active_axis_by_its_magnitude() {
        let node = make_node(car_scale());
        let bus = bus_for(&node);
        write_intent(
            &bus,
            TwistIntent {
                surge: 1.0,
                yaw: 1.0,
                ..TwistIntent::neutral()
            },
            1.0,
        );

        node.execute(&bus, &MockRuntime, tick_at(1.0));

        let out = bus.read::<BodyTwist>(output_key()).unwrap();
        assert_eq!(out.value.linear(), FluVector::new(4.0, 0.0, 0.0));
        assert_eq!(out.value.angular(), FluVector::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn inactive_scale_zeroes_its_axis_reducing_to_unicycle() {
        // Every intent axis fully deflected, but the car scale is zero off the
        // unicycle DOF: sway / heave / roll / pitch must not reach the output.
        let node = make_node(car_scale());
        let bus = bus_for(&node);
        write_intent(
            &bus,
            TwistIntent {
                surge: 1.0,
                sway: 1.0,
                heave: 1.0,
                roll: 1.0,
                pitch: 1.0,
                yaw: 1.0,
            },
            1.0,
        );

        node.execute(&bus, &MockRuntime, tick_at(1.0));

        let out = bus.read::<BodyTwist>(output_key()).unwrap();
        assert_eq!(out.value.linear(), FluVector::new(4.0, 0.0, 0.0));
        assert_eq!(out.value.angular(), FluVector::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn output_carries_the_intent_timestamp_not_the_tick() {
        // Freshness must reflect when the operator acted, so a stale teleop
        // command can yield to autonomy. Intent stamped at 3.0, ticked at 10.0 —
        // the output must read 3.0, and be produced by this node.
        let node = make_node(car_scale());
        let bus = bus_for(&node);
        write_intent(
            &bus,
            TwistIntent {
                surge: 1.0,
                ..TwistIntent::neutral()
            },
            3.0,
        );

        node.execute(&bus, &MockRuntime, tick_at(10.0));

        let out = bus.read::<BodyTwist>(output_key()).unwrap();
        assert!((out.timestamp.0 - 3.0).abs() < 1e-9);
        assert_eq!(out.producer, 7);
    }

    #[test]
    fn absent_intent_publishes_nothing() {
        // Cold start, or a host publishing only on active input: no intent on the
        // bus, so no command is written and the actuator deadman takes over.
        let node = make_node(car_scale());
        let bus = bus_for(&node);

        node.execute(&bus, &MockRuntime, tick_at(1.0));

        assert!(bus.read::<BodyTwist>(output_key()).is_none());
    }

    #[test]
    fn out_of_range_intent_is_clamped_before_scaling() {
        // A device overshooting the [-1, 1] contract cannot drive past the
        // configured magnitude: surge 2.5 clamps to 1.0, so the output is 4.0.
        let node = make_node(car_scale());
        let bus = bus_for(&node);
        write_intent(
            &bus,
            TwistIntent {
                surge: 2.5,
                ..TwistIntent::neutral()
            },
            1.0,
        );

        node.execute(&bus, &MockRuntime, tick_at(1.0));

        let out = bus.read::<BodyTwist>(output_key()).unwrap();
        assert_eq!(out.value.linear(), FluVector::new(4.0, 0.0, 0.0));
    }

    #[test]
    fn descriptor_has_intent_input_and_twist_output() {
        let node = make_node(car_scale());
        let d = node.port_descriptor();
        assert_eq!(d.required_inputs, vec![intent_key()]);
        assert_eq!(d.outputs, vec![output_key()]);
        assert!(d.rate.is_none());
    }
}
