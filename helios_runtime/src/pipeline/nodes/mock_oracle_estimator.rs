//! [`MockOracleEstimatorNode`] — passthrough "estimator" that republishes the
//! body's oracle pose (and twist, when available) as a `FrameAwareState`.
//!
//! No filter math. Used to skip estimation entirely for controller / planner
//! bring-up, demos where estimation drift would distract, and unit tests of
//! downstream stages.
//!
//! ## Execution
//!
//! 1. Read `oracle/pose`. If absent → cold-start, return (downstream sees the
//!    last-known-good `FrameAwareState` from the previous tick, if any).
//! 2. Read `oracle/twist` (optional).
//! 3. Build a `FrameAwareState` on the standard INS layout from the pose
//!    and twist; bias slots are zero-filled — this node has no way to know
//!    them.
//! 4. Publish on `Internal<FrameAwareState>` (default-named slot).
//!
//! ## Body capability requirement
//!
//! The descriptor declares `oracle/pose` as a required oracle input. A body
//! that does not publish `oracle/pose` fails the build with
//! [`PipelineBuildError::UnsatisfiedBodyCapabilities`]. This is the
//! type-level fence from `body_contract.md §9`.
//!
//! ## Known wart: zero-filled bias slots
//!
//! The standard INS layout includes accel- and gyro-bias entries. Real EKFs
//! estimate these online; this mock has no source of truth for them and
//! writes zeros. Consumers that read bias slots (none today, but possible
//! in the future) will see structurally valid but meaningless values.
//!
//! Tracked: `rollout_plan.md` Phase 11 — estimator output channel split.
//! Once `FrameAwareState` is decomposed into per-quantity channels
//! (`Pose`, `Velocity`, `PoseCovariance`, `ImuBias`), this node will simply
//! omit the bias channel.
//!
//! [`PipelineBuildError::UnsatisfiedBodyCapabilities`]:
//!     crate::pipeline::build_error::PipelineBuildError::UnsatisfiedBodyCapabilities

use crate::channels::{oracle_pose_channel, oracle_twist_channel};
use crate::pipeline::descriptor::MockNodePortDescriptor;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{InternalChannel, OracleChannel, PortBus, PortDescriptor};
use crate::runtime::AgentRuntime;
use crate::stamped::{Health, Stamped};

use helios_core::data::messages::Twist;
use helios_core::data::primitives::FrameHandle;
use helios_core::frames::layout::standard_ins_state_layout;
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};

use nalgebra::Isometry3;

pub struct MockOracleEstimatorNode {
    name: String,
    agent_handle: FrameHandle,
    descriptor: PortDescriptor,
    /// Cached at construction. `standard_ins_state_layout` allocates;
    /// rebuilding it per tick at 200 Hz × N agents would be wasteful and
    /// the layout never changes for the lifetime of the node.
    layout: Vec<StateVariable>,
}

impl MockOracleEstimatorNode {
    pub fn new(name: impl Into<String>, agent_handle: FrameHandle) -> Self {
        let descriptor = MockNodePortDescriptor::new()
            .input_oracle(OracleChannel::named::<Isometry3<f64>>("oracle/pose"))
            .optional_oracle(OracleChannel::named::<Twist>("oracle/twist"))
            .output_internal(InternalChannel::of::<FrameAwareState>())
            .build();
        Self {
            name: name.into(),
            agent_handle,
            descriptor,
            layout: standard_ins_state_layout(agent_handle),
        }
    }
}

impl PipelineNode for MockOracleEstimatorNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, _runtime: &dyn AgentRuntime, tick: TickContext) {
        // Cold-start: no oracle/pose yet → skip. The output slot keeps its
        // previous value under last-known-good semantics, or stays empty
        // if this is the first tick.
        let Some(pose_stamped) = bus.read::<Isometry3<f64>>(oracle_pose_channel()) else {
            return;
        };

        let twist = bus
            .read::<Twist>(oracle_twist_channel())
            .map(|s| s.value.clone());

        let mut state = FrameAwareState::new(self.layout.clone(), 0.0, tick.now.0);

        write_pose_into(&mut state, &pose_stamped.value, self.agent_handle);

        if let Some(t) = twist {
            // oracle/twist is world ENU and the standard INS layout's
            // velocity slots are world too — straight passthrough.
            write_world_velocity_into(&mut state, &t);
        }

        let stamped = Stamped {
            value: state,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };

        let _ = bus.write(InternalChannel::of::<FrameAwareState>().into(), stamped);
    }
}

fn write_pose_into(state: &mut FrameAwareState, pose: &Isometry3<f64>, agent: FrameHandle) {
    // Position in world frame.
    state.set_variable(&StateVariable::Px(FrameId::World), pose.translation.x);
    state.set_variable(&StateVariable::Py(FrameId::World), pose.translation.y);
    state.set_variable(&StateVariable::Pz(FrameId::World), pose.translation.z);

    // Body→world orientation as a quaternion.
    state.set_variable(
        &StateVariable::Qx(FrameId::Body(agent), FrameId::World),
        pose.rotation.i,
    );
    state.set_variable(
        &StateVariable::Qy(FrameId::Body(agent), FrameId::World),
        pose.rotation.j,
    );
    state.set_variable(
        &StateVariable::Qz(FrameId::Body(agent), FrameId::World),
        pose.rotation.k,
    );
    state.set_variable(
        &StateVariable::Qw(FrameId::Body(agent), FrameId::World),
        pose.rotation.w,
    );
}

/// Writes the world-frame linear velocity from `twist` into the state's
/// world-frame velocity slots (Vx/Vy/Vz in `FrameId::World`).
///
/// The standard INS layout has no instantaneous angular velocity slots —
/// `Wx/Wy/Wz(Body)` in that layout represent **gyro biases**, not angular
/// velocity. So `twist.angular` is dropped here on purpose. Once the
/// estimator-output split lands (Phase 11), angular velocity gets its own
/// channel and a downstream consumer can read it directly from
/// `oracle/twist` instead of routing it through `FrameAwareState`.
fn write_world_velocity_into(state: &mut FrameAwareState, twist: &Twist) {
    state.set_variable(&StateVariable::Vx(FrameId::World), twist.linear.x);
    state.set_variable(&StateVariable::Vy(FrameId::World), twist.linear.y);
    state.set_variable(&StateVariable::Vz(FrameId::World), twist.linear.z);
}

#[cfg(test)]
mod tests {
    //! Tests for [`MockOracleEstimatorNode`]:
    //! 1. Descriptor shape — oracle inputs + FrameAwareState output.
    //! 2. Execute round-trip — pose + twist published correctly.
    //! 3. Cold-start — no oracle → no publish.
    //! 4. Body-capability gate — build fails without `oracle/pose`,
    //!    succeeds with it. Locks the headline guarantee from
    //!    `mock_catalog.md §2.1`.

    use super::*;
    use crate::body::{BodyCapabilities, Provenance, PublishedChannel};
    use crate::pipeline::autonomy_pipeline::PipelineBuilder;
    use crate::pipeline::build_error::PipelineBuildError;
    use crate::port::{ChannelKey, PortDescriptor};

    use helios_core::data::primitives::MonotonicTime;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    // --- Test fixtures ---

    struct MockRuntime {
        now: f64,
    }

    impl AgentRuntime for MockRuntime {
        fn get_transform(&self, _: FrameHandle, _: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn world_pose(&self, _: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn now(&self) -> MonotonicTime {
            MonotonicTime(self.now)
        }
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 0,
        }
    }

    fn state_channel() -> ChannelKey {
        InternalChannel::of::<FrameAwareState>().into()
    }

    /// A descriptor that "produces" the oracle channels, so the bus has
    /// slots for them. The mock node depends on these slots existing —
    /// without a producer the bus skips the slot allocation and the
    /// test would silently no-op.
    fn oracle_producer_descriptor() -> PortDescriptor {
        PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![oracle_pose_channel(), oracle_twist_channel()],
            rate: None,
        }
    }

    fn make_bus_with_oracle_producer(node: &MockOracleEstimatorNode) -> PortBus {
        let producer = oracle_producer_descriptor();
        PortBus::new([node.port_descriptor(), &producer])
    }

    fn body_with_oracle() -> BodyCapabilities {
        BodyCapabilities {
            name: "test_body".to_string(),
            publishes: vec![
                PublishedChannel {
                    key: oracle_pose_channel(),
                    provenance: Provenance::Exact,
                },
                PublishedChannel {
                    key: oracle_twist_channel(),
                    provenance: Provenance::Exact,
                },
            ],
            consumes_control: true,
        }
    }

    // --- Descriptor shape ---

    #[test]
    fn descriptor_requires_oracle_pose() {
        let node = MockOracleEstimatorNode::new("mock", FrameHandle(1));
        assert!(
            node.port_descriptor()
                .required_inputs
                .contains(&oracle_pose_channel()),
            "oracle/pose must be a required input"
        );
    }

    #[test]
    fn descriptor_marks_oracle_twist_optional() {
        let node = MockOracleEstimatorNode::new("mock", FrameHandle(1));
        assert!(
            node.port_descriptor()
                .optional_inputs
                .contains(&oracle_twist_channel()),
            "oracle/twist must be optional, not required"
        );
    }

    #[test]
    fn descriptor_outputs_frame_aware_state() {
        let node = MockOracleEstimatorNode::new("mock", FrameHandle(1));
        assert_eq!(node.port_descriptor().outputs, vec![state_channel()]);
    }

    // --- Execute behavior ---

    #[test]
    fn republishes_pose_as_frame_aware_state() {
        let agent = FrameHandle(1);
        let node = MockOracleEstimatorNode::new("mock", agent);
        let bus = make_bus_with_oracle_producer(&node);

        // Write a known pose to oracle/pose.
        let pose = Isometry3::from_parts(
            Translation3::new(1.5, -2.0, 0.5),
            UnitQuaternion::identity(),
        );
        bus.write(
            oracle_pose_channel(),
            Stamped {
                value: pose,
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 99,
            },
        )
        .expect("oracle/pose slot must exist");

        node.execute(&bus, &MockRuntime { now: 0.0 }, tick_at(0.0, 0.01));

        let published = bus
            .read::<FrameAwareState>(state_channel())
            .expect("node must publish FrameAwareState");
        let recovered = published
            .value
            .get_pose_isometry()
            .expect("standard layout includes pose");
        let dx = (recovered.translation.vector - pose.translation.vector).norm();
        assert!(
            dx < 1e-9,
            "pose translation should round-trip exactly, dx={dx}"
        );
    }

    #[test]
    fn republishes_world_velocity_into_state_world_slots() {
        // oracle/twist is already world-frame; mock just passes it through
        // into the world-frame velocity slots of FrameAwareState.
        let agent = FrameHandle(1);
        let node = MockOracleEstimatorNode::new("mock", agent);
        let bus = make_bus_with_oracle_producer(&node);

        bus.write(
            oracle_pose_channel(),
            Stamped {
                value: Isometry3::<f64>::identity(),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 99,
            },
        )
        .expect("oracle/pose slot must exist");

        let twist = Twist {
            linear: Vector3::new(2.0, -1.0, 0.5),
            // angular has no slot in the standard INS layout — see
            // write_world_velocity_into's doc. Set non-zero anyway to
            // make sure execute() doesn't blow up on it.
            angular: Vector3::new(0.0, 0.0, 0.3),
        };
        bus.write(
            oracle_twist_channel(),
            Stamped {
                value: twist.clone(),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 99,
            },
        )
        .expect("oracle/twist slot must exist");

        node.execute(&bus, &MockRuntime { now: 0.0 }, tick_at(0.0, 0.01));

        let out = bus
            .read::<FrameAwareState>(state_channel())
            .expect("must publish");

        // Lookup-by-pattern avoids hardcoding the index — the layout's
        // ordering is not a stable contract.
        let world_v = |axis: char| {
            let pred: fn(&StateVariable) -> bool = match axis {
                'x' => |v| matches!(v, StateVariable::Vx(FrameId::World)),
                'y' => |v| matches!(v, StateVariable::Vy(FrameId::World)),
                'z' => |v| matches!(v, StateVariable::Vz(FrameId::World)),
                _ => unreachable!(),
            };
            let idx = out
                .value
                .state
                .layout
                .iter()
                .position(pred)
                .expect("layout has Vx/Vy/Vz(World)");
            out.value.state.vector[idx]
        };

        assert!((world_v('x') - 2.0).abs() < 1e-9);
        assert!((world_v('y') - -1.0).abs() < 1e-9);
        assert!((world_v('z') - 0.5).abs() < 1e-9);
    }

    #[test]
    fn skips_publish_when_oracle_pose_absent() {
        let node = MockOracleEstimatorNode::new("mock", FrameHandle(1));
        let bus = make_bus_with_oracle_producer(&node);

        // No write to oracle/pose. Cold start.
        node.execute(&bus, &MockRuntime { now: 0.0 }, tick_at(0.0, 0.01));

        assert!(
            bus.read::<FrameAwareState>(state_channel()).is_none(),
            "must not publish without oracle/pose"
        );
    }

    #[test]
    fn stamp_uses_tick_now_and_node_id() {
        let node = MockOracleEstimatorNode::new("mock", FrameHandle(1));
        let bus = make_bus_with_oracle_producer(&node);
        bus.write(
            oracle_pose_channel(),
            Stamped {
                value: Isometry3::<f64>::identity(),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 99,
            },
        )
        .unwrap();

        let tick = TickContext {
            now: MonotonicTime(3.5),
            dt: 0.01,
            node_id: 7,
        };
        node.execute(&bus, &MockRuntime { now: 3.5 }, tick);

        let out = bus.read::<FrameAwareState>(state_channel()).unwrap();
        assert!((out.timestamp.0 - 3.5).abs() < 1e-9);
        assert_eq!(out.producer, 7);
    }

    // --- Build-time body-capability gate ---

    #[test]
    fn build_fails_when_body_lacks_oracle_pose() {
        let node = MockOracleEstimatorNode::new("mock", FrameHandle(1));
        let empty_body = BodyCapabilities {
            name: "no_oracle_body".to_string(),
            publishes: vec![],
            consumes_control: true,
        };

        // .map(|_| ()) drops the AutonomyPipeline so expect_err / Debug
        // formatting work — AutonomyPipeline doesn't implement Debug.
        let errs = PipelineBuilder::new()
            .add_node(Box::new(node))
            .with_body_capabilities(empty_body)
            .build()
            .map(|_| ())
            .expect_err("build must reject mock without oracle/pose");

        let saw = errs.iter().any(|e| {
            matches!(
                e,
                PipelineBuildError::UnsatisfiedBodyCapabilities { body, channel_key, .. }
                    if body == "no_oracle_body" && *channel_key == oracle_pose_channel()
            )
        });
        assert!(
            saw,
            "expected UnsatisfiedBodyCapabilities for oracle/pose, got: {errs:?}"
        );
    }

    #[test]
    fn build_succeeds_when_body_publishes_oracle_pose() {
        let node = MockOracleEstimatorNode::new("mock", FrameHandle(1));

        let result = PipelineBuilder::new()
            .add_node(Box::new(node))
            .with_body_capabilities(body_with_oracle())
            .build()
            .map(|_| ());

        assert!(result.is_ok(), "build should succeed: {result:?}");
    }
}
