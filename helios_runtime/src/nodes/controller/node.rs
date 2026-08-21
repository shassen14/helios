//! [`ControllerNode`] — pipeline adapter for any [`Controller`] implementation.
//!
//! One node type for the whole controller family. The node is **generic over the
//! controller `C`**: it holds the concrete controller inline (`Mutex<C>`) and
//! publishes `C::Out`, the controller's concrete command type (e.g. `BodyTwist`).
//! Type erasure happens one level up — the assembler boxes the monomorphized
//! `ControllerNode<C>` as `Box<dyn PipelineNode>`. Because `Controller` carries
//! associated types (`Inputs`, `Out`) it is not object-safe, so there is no
//! `Box<dyn Controller>`; the DAG's typed channel *is* the command-space contract.
//!
//! ## Execution skeleton
//!
//! 1. Ask the [`ControlInputBuilder`] to assemble [`ControlInputs`] from the bus.
//!    On `None` (state missing — cold-start, estimator dropout), publish nothing
//!    this tick. Downstream actuators fall back to their no-command behaviour.
//! 2. Run [`Controller::compute`] with the assembled inputs.
//! 3. Publish a `Stamped<C::Out>` on the caller-supplied output channel (the
//!    assembler routes it to the command selector's autonomy input).
//!
//! ## Cold-start vs. missing-reference
//!
//! `DefaultControlInputBuilder` returns `None` only when `FrameAwareState` is
//! absent. A `Some(ControlInputs { state, reference: None })` is still passed
//! to the controller — each implementation decides what to do (e.g.
//! `DirectTwistController` emits a zero twist when no reference is present;
//! a controller holding feedback state would degrade to pure feedback).
//! Publishing a safe-zero on full cold-start belongs in a future
//! `SafetyMonitorNode` downstream of this node, not here.

use super::input::ControlInputBuilder;
use crate::pipeline::descriptor::AlgorithmNodePortDescriptor;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, InternalChannel, PortBus, PortDescriptor};
use crate::runtime::AgentRuntime;
use crate::stamped::{Health, Stamped};

use helios_core::control::Controller;
use helios_core::prelude::ControlInputs;

use std::sync::Mutex;

/// Pipeline node wrapping any [`Controller`] implementation.
///
/// Construction is via [`Self::new`]. The port descriptor's required and
/// optional inputs are taken directly from the input builder; the single output
/// is the caller-supplied channel, typed `C::Out` — the controller's concrete
/// command type. `rate` is `None` — controllers fire every tick (rate-limiting
/// belongs to the actuator side if it's needed at all).
pub(crate) struct ControllerNode<C: Controller> {
    name: String,
    controller: Mutex<C>,
    input_builder: Box<dyn ControlInputBuilder>,
    descriptor: PortDescriptor,
    output_key: ChannelKey,
}

impl<C> ControllerNode<C>
where
    C: Controller<Inputs = ControlInputs>,
    C::Out: Send + Sync + 'static,
{
    pub(crate) fn new(
        name: impl Into<String>,
        controller: C,
        input_builder: Box<dyn ControlInputBuilder>,
        output: InternalChannel,
    ) -> Self {
        let descriptor = AlgorithmNodePortDescriptor::new()
            .inputs_from_slices(
                input_builder.required_channels(),
                input_builder.optional_channels(),
            )
            .output_internal(output.clone())
            .build();

        Self {
            name: name.into(),
            controller: Mutex::new(controller),
            input_builder,
            descriptor,
            output_key: output.into(),
        }
    }
}

impl<C> PipelineNode for ControllerNode<C>
where
    C: Controller<Inputs = ControlInputs>,
    C::Out: Send + Sync + 'static,
{
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, runtime: &dyn AgentRuntime, tick: TickContext) {
        // Cold-start / estimator dropout: state absent → nothing to control.
        let Some(inputs) = self.input_builder.assemble(bus, runtime, &tick) else {
            return;
        };

        // Skip the tick on a poisoned mutex rather than propagating the panic.
        let Ok(mut controller) = self.controller.lock() else {
            return;
        };

        let output = controller.compute(tick.dt, &inputs);

        let stamped = Stamped {
            value: output,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(self.output_key.clone(), stamped);
    }
}

#[cfg(test)]
mod tests {
    //! Wiring tests for [`ControllerNode`] — concrete controller behaviour
    //! (control laws, gains) is covered in `helios_core/src/control/`. Here we
    //! verify that `execute()`:
    //!   - publishes a `Stamped<C::Out>` with correct timestamp / producer
    //!   - routes the output onto the `C::Out`-typed channel (proving the node is
    //!     generic over the command type, not pinned to one output)
    //!   - early-returns (publishes nothing) when the input builder yields `None`
    //!   - forwards `dt` to `Controller::compute`
    //!   - mirrors the builder's required/optional channels in its descriptor

    use super::*;
    use helios_core::frames::transforms::{Convention, ErasedTransform};

    use crate::port::ChannelKey;

    use helios_core::control::commands::{BodyTwist, BodyWrench};
    use helios_core::data::primitives::{FrameHandle, MonotonicTime};
    use helios_core::estimation::carrier::kinematic_carrier_schema;
    use helios_core::frames::quantities::FluVector;
    use helios_core::frames::{FrameAwareState, FrameId};

    use nalgebra::Isometry3;
    use std::sync::{Arc, Mutex as StdMutex};

    // --- Mock AgentRuntime ---

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

    // --- Mock Controller that records dt and returns a scripted BodyTwist ---
    //
    // `Out = BodyTwist` stands in for the current controller family. A second
    // controller below uses `Out = BodyWrench` to prove the node is generic over
    // the command type, not wired to a single output.

    #[derive(Default)]
    struct MockControllerCalls {
        compute_calls: u32,
        last_dt: f64,
    }

    struct ScriptedController {
        calls: Arc<StdMutex<MockControllerCalls>>,
    }

    impl ScriptedController {
        fn new() -> (Self, Arc<StdMutex<MockControllerCalls>>) {
            let calls = Arc::new(StdMutex::new(MockControllerCalls::default()));
            (
                Self {
                    calls: Arc::clone(&calls),
                },
                calls,
            )
        }
    }

    impl Controller for ScriptedController {
        type Inputs = ControlInputs;
        type Out = BodyTwist;

        fn compute(&mut self, dt: f64, _inputs: &ControlInputs) -> BodyTwist {
            let mut c = self.calls.lock().unwrap();
            c.compute_calls += 1;
            c.last_dt = dt;
            BodyTwist::new(FluVector::new(1.0, 0.0, 0.0), FluVector::zeros())
        }
        fn reset(&mut self) {}
    }

    // A controller emitting a different command type, used only to show that the
    // node monomorphizes over `C::Out` and publishes on the matching channel.
    struct WrenchController;

    impl Controller for WrenchController {
        type Inputs = ControlInputs;
        type Out = BodyWrench;

        fn compute(&mut self, _dt: f64, _inputs: &ControlInputs) -> BodyWrench {
            BodyWrench::zero()
        }
        fn reset(&mut self) {}
    }

    // --- Mock ControlInputBuilders ---

    struct AlwaysReadyBuilder {
        required: Vec<ChannelKey>,
        optional: Vec<ChannelKey>,
    }
    impl AlwaysReadyBuilder {
        fn new() -> Self {
            Self {
                required: vec![InternalChannel::of::<FrameAwareState>().into()],
                optional: vec![],
            }
        }
    }
    impl ControlInputBuilder for AlwaysReadyBuilder {
        fn assemble(
            &self,
            _bus: &PortBus,
            _runtime: &dyn AgentRuntime,
            _tick: &TickContext,
        ) -> Option<ControlInputs> {
            Some(ControlInputs {
                // A placeholder kinematic state; this mock never reads its contents.
                state: FrameAwareState::from_schema(
                    std::sync::Arc::new(kinematic_carrier_schema(FrameHandle(0))),
                    0.0,
                ),
                reference: None,
            })
        }
        fn required_channels(&self) -> &[ChannelKey] {
            &self.required
        }
        fn optional_channels(&self) -> &[ChannelKey] {
            &self.optional
        }
    }

    struct NeverReadyBuilder {
        required: Vec<ChannelKey>,
        optional: Vec<ChannelKey>,
    }
    impl ControlInputBuilder for NeverReadyBuilder {
        fn assemble(
            &self,
            _bus: &PortBus,
            _runtime: &dyn AgentRuntime,
            _tick: &TickContext,
        ) -> Option<ControlInputs> {
            None
        }
        fn required_channels(&self) -> &[ChannelKey] {
            &self.required
        }
        fn optional_channels(&self) -> &[ChannelKey] {
            &self.optional
        }
    }

    // --- Helpers ---

    fn twist_channel() -> ChannelKey {
        InternalChannel::of::<BodyTwist>().into()
    }

    fn make_bus(output: ChannelKey) -> PortBus {
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![output],
            rate: None,
        };
        PortBus::new(&[descriptor])
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 11,
        }
    }

    // --- Tests ---

    #[test]
    fn descriptor_outputs_the_controllers_command_channel() {
        let (controller, _calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "direct_twist",
            controller,
            Box::new(AlwaysReadyBuilder::new()),
            InternalChannel::of::<BodyTwist>(),
        );
        assert_eq!(node.port_descriptor().outputs, vec![twist_channel()]);
        assert!(node.port_descriptor().rate.is_none());
    }

    #[test]
    fn descriptor_mirrors_builder_required_and_optional_channels() {
        let builder = AlwaysReadyBuilder::new();
        let expected_required = builder.required_channels().to_vec();
        let expected_optional = builder.optional_channels().to_vec();

        let (controller, _calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "direct_twist",
            controller,
            Box::new(builder),
            InternalChannel::of::<BodyTwist>(),
        );
        assert_eq!(node.port_descriptor().required_inputs, expected_required);
        assert_eq!(node.port_descriptor().optional_inputs, expected_optional);
    }

    #[test]
    fn execute_publishes_command_with_correct_stamp_and_value() {
        let (controller, _calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "direct_twist",
            controller,
            Box::new(AlwaysReadyBuilder::new()),
            InternalChannel::of::<BodyTwist>(),
        );
        let bus = make_bus(twist_channel());
        node.execute(&bus, &MockRuntime, tick_at(2.5, 0.05));

        let published = bus
            .read::<BodyTwist>(twist_channel())
            .expect("node must publish its command when inputs are ready");
        assert!((published.timestamp.0 - 2.5).abs() < 1e-9);
        assert_eq!(published.producer, 11);
        assert_eq!(published.value.linear(), FluVector::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn node_is_generic_over_output_type() {
        // The same node type, monomorphized over a controller whose `Out` is
        // `BodyWrench`, publishes on a `BodyWrench`-typed channel — the output
        // slot is caller-supplied but its payload type must match `C::Out`.
        let wrench_channel: ChannelKey = InternalChannel::of::<BodyWrench>().into();
        let node = ControllerNode::new(
            "wrench",
            WrenchController,
            Box::new(AlwaysReadyBuilder::new()),
            InternalChannel::of::<BodyWrench>(),
        );
        assert_eq!(node.port_descriptor().outputs, vec![wrench_channel.clone()]);

        let bus = make_bus(wrench_channel.clone());
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<BodyWrench>(wrench_channel).is_some());
    }

    #[test]
    fn execute_early_returns_when_input_builder_returns_none() {
        // Cold-start: builder reports state is missing → publish nothing,
        // and the controller is never called.
        let (controller, calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "direct_twist",
            controller,
            Box::new(NeverReadyBuilder {
                required: vec![],
                optional: vec![],
            }),
            InternalChannel::of::<BodyTwist>(),
        );
        let bus = make_bus(twist_channel());
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<BodyTwist>(twist_channel()).is_none());
        assert_eq!(calls.lock().unwrap().compute_calls, 0);
    }

    #[test]
    fn execute_forwards_dt_to_controller() {
        let (controller, calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "direct_twist",
            controller,
            Box::new(AlwaysReadyBuilder::new()),
            InternalChannel::of::<BodyTwist>(),
        );
        let bus = make_bus(twist_channel());
        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.02));
        node.execute(&bus, &MockRuntime, tick_at(0.02, 0.04));

        let calls = calls.lock().unwrap();
        assert_eq!(calls.compute_calls, 2);
        assert!((calls.last_dt - 0.04).abs() < 1e-9);
    }
}
