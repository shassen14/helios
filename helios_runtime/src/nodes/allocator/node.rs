use crate::pipeline::descriptor::AlgorithmNodePortDescriptor;
use crate::port::{ChannelError, InternalChannel, PortBus};
use crate::{AgentRuntime, ChannelKey, Health, PipelineNode, PortDescriptor, Stamped, TickContext};

use helios_core::control::allocation::Allocator;

use std::sync::Mutex;

pub(crate) struct AllocatorNode<A: Allocator> {
    name: String,
    allocator: Mutex<A>,
    descriptor: PortDescriptor,
    input_key: ChannelKey,
    output_key: ChannelKey,
}

impl<A: Allocator> AllocatorNode<A> {
    pub fn new(
        name: impl Into<String>,
        allocator: A,
        input: InternalChannel,
        output: InternalChannel,
    ) -> Self {
        let descriptor = AlgorithmNodePortDescriptor::new()
            .input_internal(input.clone())
            .output_internal(output.clone())
            .build();

        Self {
            name: name.into(),
            allocator: Mutex::new(allocator),
            descriptor,
            input_key: input.into(),
            output_key: output.into(),
        }
    }
}

impl<A> PipelineNode for AllocatorNode<A>
where
    A: Allocator<Inputs = ()>,
    A::In: Send + Sync + 'static,
{
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, _runtime: &dyn AgentRuntime, tick: TickContext) {
        let Some(command) = bus.read::<A::In>(self.input_key.clone()) else {
            return;
        };

        let Ok(mut allocator) = self.allocator.lock() else {
            return;
        };

        let out = allocator.allocate(&command.value, &());

        if let Err(ChannelError::UnknownChannel) = bus.write(
            self.output_key.clone(),
            Stamped {
                value: out,
                timestamp: tick.now,
                health: Health::Ok,
                producer: tick.node_id,
            },
        ) {
            tracing::warn!(channel = %self.output_key, "allocator output channel is not wired into the DAG");
        }
    }
}

#[cfg(test)]
mod tests {
    //! Wiring tests for [`AllocatorNode`] — the allocation math itself (the
    //! bicycle inverse) is covered in `helios_core/src/control/allocation/`. Here
    //! we verify that `execute()`:
    //!   - publishes a `Stamped<ActuatorCommand>` with the right timestamp /
    //!     producer on the terminal channel
    //!   - reads the command off its input channel and passes it to `allocate`
    //!   - early-returns (publishes nothing, never calls `allocate`) when no
    //!     command is present — cold-start / upstream dropout
    //!   - mirrors its command input and actuator output in its descriptor

    use super::*;

    use helios_core::control::actuators::{
        ActuatorCommand, ActuatorId, ActuatorSetpoint, SetpointValue,
    };
    use helios_core::control::commands::BodyTwist;
    use helios_core::data::primitives::{FrameHandle, MonotonicTime};
    use helios_core::frames::conventions::FluVector;

    use nalgebra::Isometry3;
    use std::sync::{Arc, Mutex as StdMutex};

    // --- Mock AgentRuntime (the allocator ignores it, but `execute` needs one) ---

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

    // --- Stub allocator: records the forward speed it saw, returns a canned command ---

    struct StubAllocator {
        output: ActuatorCommand,
        seen_vel_x: Arc<StdMutex<Option<f64>>>,
    }

    impl Allocator for StubAllocator {
        type In = BodyTwist;
        type Inputs = ();

        fn allocate(&mut self, command: &BodyTwist, _inputs: &()) -> ActuatorCommand {
            *self.seen_vel_x.lock().unwrap() = Some(command.linear().x());
            self.output.clone()
        }
    }

    // --- Helpers ---

    fn command_channel() -> InternalChannel {
        InternalChannel::of::<BodyTwist>()
    }

    fn actuator_channel() -> InternalChannel {
        InternalChannel::of::<ActuatorCommand>()
    }

    fn canned_command() -> ActuatorCommand {
        ActuatorCommand::new(vec![ActuatorSetpoint::new(
            ActuatorId::new("drive"),
            SetpointValue::Velocity(5.0),
        )])
    }

    fn node(seen: Arc<StdMutex<Option<f64>>>) -> AllocatorNode<StubAllocator> {
        AllocatorNode::new(
            "ackermann".to_string(),
            StubAllocator {
                output: canned_command(),
                seen_vel_x: seen,
            },
            command_channel(),
            actuator_channel(),
        )
    }

    fn make_bus(node: &AllocatorNode<StubAllocator>) -> PortBus {
        PortBus::new([node.port_descriptor()])
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 7,
        }
    }

    fn write_command(bus: &PortBus, vel_x: f64) {
        let twist = BodyTwist::new(FluVector::new(vel_x, 0.0, 0.0), FluVector::zeros());
        bus.write(
            command_channel().into(),
            Stamped {
                value: twist,
                timestamp: MonotonicTime(1.0),
                health: Health::Ok,
                producer: 1,
            },
        )
        .expect("command channel slot exists");
    }

    // --- Tests ---

    #[test]
    fn descriptor_reads_command_and_outputs_actuator_command() {
        let node = node(Arc::new(StdMutex::new(None)));
        assert_eq!(
            node.port_descriptor().required_inputs,
            vec![command_channel().into()]
        );
        assert_eq!(
            node.port_descriptor().outputs,
            vec![actuator_channel().into()]
        );
        assert!(node.port_descriptor().rate.is_none());
    }

    #[test]
    fn execute_publishes_actuator_command_with_stamp_and_value() {
        let node = node(Arc::new(StdMutex::new(None)));
        let bus = make_bus(&node);
        write_command(&bus, 3.0);

        node.execute(&bus, &MockRuntime, tick_at(2.5, 0.05));

        let published = bus
            .read::<ActuatorCommand>(actuator_channel().into())
            .expect("allocator must publish when a command is present");
        assert!((published.timestamp.0 - 2.5).abs() < 1e-9);
        assert_eq!(published.producer, 7);
        assert_eq!(published.value, canned_command());
    }

    #[test]
    fn execute_forwards_command_to_allocator() {
        // The node reads its input channel and hands that command to `allocate` —
        // proving the command flows through, not just that something is published.
        let seen = Arc::new(StdMutex::new(None));
        let node = node(Arc::clone(&seen));
        let bus = make_bus(&node);
        write_command(&bus, 4.2);

        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.02));

        assert_eq!(*seen.lock().unwrap(), Some(4.2));
    }

    #[test]
    fn execute_early_returns_when_command_absent() {
        // Cold-start / upstream dropout: no command on the bus → publish nothing,
        // and `allocate` is never called.
        let seen = Arc::new(StdMutex::new(None));
        let node = node(Arc::clone(&seen));
        let bus = make_bus(&node);

        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));

        assert!(bus
            .read::<ActuatorCommand>(actuator_channel().into())
            .is_none());
        assert!(
            seen.lock().unwrap().is_none(),
            "allocate must not run without a command"
        );
    }
}
