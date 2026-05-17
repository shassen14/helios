//! [`ControllerNode`] — pipeline adapter for any [`Controller`] implementation.
//!
//! One node type for the whole controller family today (PID, LQR, FF+PID,
//! DirectVelocity).
//!
//! ## Execution skeleton
//!
//! 1. Ask the [`ControlInputBuilder`] to assemble [`ControlInputs`] from the bus.
//!    On `None` (state missing — cold-start, estimator dropout), publish nothing
//!    this tick. Downstream actuators fall back to their no-command behaviour.
//! 2. Run [`Controller::compute`] with the assembled inputs.
//! 3. Publish a `Stamped<ControlOutput>` on `ChannelKey::of::<ControlOutput>()`.
//!
//! ## Cold-start vs. missing-reference
//!
//! `DefaultControlInputBuilder` returns `None` only when `FrameAwareState` is
//! absent. A `Some(ControlInputs { state, reference: None })` is still passed
//! to the controller — each implementation decides what to do (DirectVelocity
//! emits zero velocity; PID-family controllers degrade to pure feedback).
//! Publishing a safe-zero on full cold-start belongs in a future
//! `SafetyMonitorNode` downstream of this node, not here.

use std::sync::Mutex;

use helios_core::control::{ControlOutput, Controller};

use crate::pipeline::builders::controller::ControlInputBuilder;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, PortBus, PortDescriptor};
use crate::runtime::AgentRuntime;
use crate::stamped::{Health, Stamped};

/// Pipeline node wrapping any [`Controller`] implementation.
///
/// Construction is via [`Self::new`]. The port descriptor's required and
/// optional inputs are taken directly from the input builder; output is
/// always `ControlOutput @ ""`. `rate` is `None` — controllers fire every
/// tick (rate-limiting belongs to the actuator side if it's needed at all).
pub struct ControllerNode {
    name: String,
    controller: Mutex<Box<dyn Controller>>,
    input_builder: Box<dyn ControlInputBuilder>,
    descriptor: PortDescriptor,
}

impl ControllerNode {
    pub fn new(
        name: impl Into<String>,
        controller: Box<dyn Controller>,
        input_builder: Box<dyn ControlInputBuilder>,
    ) -> Self {
        let required_inputs = input_builder.required_channels().to_vec();
        let optional_inputs = input_builder.optional_channels().to_vec();
        let descriptor = PortDescriptor {
            required_inputs,
            optional_inputs,
            outputs: vec![ChannelKey::of::<ControlOutput>()],
            rate: None,
        };
        Self {
            name: name.into(),
            controller: Mutex::new(controller),
            input_builder,
            descriptor,
        }
    }
}

impl PipelineNode for ControllerNode {
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
        let _ = bus.write(ChannelKey::of::<ControlOutput>(), stamped);
    }
}

#[cfg(test)]
mod tests {
    //! Wiring tests for [`ControllerNode`] — concrete controller behaviour
    //! (PID gains, LQR Riccati, FF+PID pseudo-inverse) is covered in
    //! `helios_core/src/control/`. Here we verify that `execute()`:
    //!   - publishes a `Stamped<ControlOutput>` with correct timestamp / producer
    //!   - early-returns (publishes nothing) when the input builder yields `None`
    //!   - forwards `dt` to `Controller::compute`
    //!   - mirrors the builder's required/optional channels in its descriptor

    use super::*;

    use helios_core::control::ControlInputs;
    use helios_core::data::primitives::{FrameHandle, MonotonicTime};
    use helios_core::frames::{FrameAwareState, FrameId, StateVariable};

    use nalgebra::{Isometry3, Vector3};
    use std::sync::{Arc, Mutex as StdMutex};

    // --- Mock AgentRuntime ---

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

    // --- Mock Controller that records dt and returns a scripted output ---

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
        fn compute(&mut self, dt: f64, _inputs: &ControlInputs) -> ControlOutput {
            let mut c = self.calls.lock().unwrap();
            c.compute_calls += 1;
            c.last_dt = dt;
            ControlOutput::BodyVelocity {
                linear: Vector3::new(1.0, 0.0, 0.0),
                angular: Vector3::zeros(),
            }
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
                required: vec![ChannelKey::of::<FrameAwareState>()],
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
                state: FrameAwareState::new(vec![StateVariable::Px(FrameId::World)], 1.0, 0.0),
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

    fn out_channel() -> ChannelKey {
        ChannelKey::of::<ControlOutput>()
    }

    fn make_bus() -> PortBus {
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![out_channel()],
            rate: None,
        };
        PortBus::new(&[descriptor], vec![])
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
    fn descriptor_outputs_control_output_channel() {
        let (controller, _calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "direct_velocity",
            Box::new(controller),
            Box::new(AlwaysReadyBuilder::new()),
        );
        assert_eq!(node.port_descriptor().outputs, vec![out_channel()]);
        assert!(node.port_descriptor().rate.is_none());
    }

    #[test]
    fn descriptor_mirrors_builder_required_and_optional_channels() {
        let builder = AlwaysReadyBuilder::new();
        let expected_required = builder.required_channels().to_vec();
        let expected_optional = builder.optional_channels().to_vec();

        let (controller, _calls) = ScriptedController::new();
        let node = ControllerNode::new("pid", Box::new(controller), Box::new(builder));
        assert_eq!(node.port_descriptor().required_inputs, expected_required);
        assert_eq!(node.port_descriptor().optional_inputs, expected_optional);
    }

    #[test]
    fn execute_publishes_control_output_with_correct_stamp() {
        let (controller, _calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "direct_velocity",
            Box::new(controller),
            Box::new(AlwaysReadyBuilder::new()),
        );
        let bus = make_bus();
        node.execute(&bus, &MockRuntime, tick_at(2.5, 0.05));

        let published = bus
            .read::<ControlOutput>(out_channel())
            .expect("node must publish ControlOutput when inputs are ready");
        assert!((published.timestamp.0 - 2.5).abs() < 1e-9);
        assert_eq!(published.producer, 11);
    }

    #[test]
    fn execute_early_returns_when_input_builder_returns_none() {
        // Cold-start: builder reports state is missing → publish nothing,
        // and the controller is never called.
        let (controller, calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "pid",
            Box::new(controller),
            Box::new(NeverReadyBuilder {
                required: vec![],
                optional: vec![],
            }),
        );
        let bus = make_bus();
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<ControlOutput>(out_channel()).is_none());
        assert_eq!(calls.lock().unwrap().compute_calls, 0);
    }

    #[test]
    fn execute_forwards_dt_to_controller() {
        let (controller, calls) = ScriptedController::new();
        let node = ControllerNode::new(
            "pid",
            Box::new(controller),
            Box::new(AlwaysReadyBuilder::new()),
        );
        let bus = make_bus();
        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.02));
        node.execute(&bus, &MockRuntime, tick_at(0.02, 0.04));

        let calls = calls.lock().unwrap();
        assert_eq!(calls.compute_calls, 2);
        assert!((calls.last_dt - 0.04).abs() < 1e-9);
    }
}
