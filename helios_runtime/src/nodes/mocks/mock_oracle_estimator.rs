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
//! 3. Build a `FrameAwareState` on the kinematic carrier schema from the pose
//!    and twist — position, linear/angular velocity, and attitude.
//! 4. Publish on the channel named after the node. Like any estimator it
//!    publishes no TF edge; the estimate seam's relay does.
//!
//! ## Body capability requirement
//!
//! The descriptor declares `oracle/pose` as a required oracle input. A body
//! that does not publish `oracle/pose` fails the build with
//! [`PipelineBuildError::UnsatisfiedBodyCapabilities`]. This is the
//! type-level fence that keeps oracle-reading nodes off bodies without truth.
//!
//! ## Schema
//!
//! The published state uses the kinematic carrier schema — exactly the
//! quantities a pose + twist truth source expresses, and no more. Unlike a real
//! INS estimate it carries no sensor-bias blocks: this node has no source of
//! truth for biases, so rather than zero-fill meaningless slots it simply does
//! not declare them.
//!
//! [`PipelineBuildError::UnsatisfiedBodyCapabilities`]:
//!     crate::pipeline::build_error::PipelineBuildError::UnsatisfiedBodyCapabilities

use crate::channels::estimate::estimator_output;
use crate::channels::{oracle_pose_channel, oracle_twist_channel};
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelError, ChannelKey, MockNodePortDescriptor, PortBus, PortDescriptor};
use crate::stamped::{Health, Stamped};

use helios_core::estimation::carrier::kinematic_carrier_schema;
use helios_core::estimation::schema::StateSchema;
use helios_core::interchange::motion::Twist;
use helios_core::prelude::AgentId;
use helios_core::prelude::TfProvider;
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::quantities::FreeVector;
use helios_core::spatial::transforms::Transform;
use helios_core::spatial::{BlockWriteError, FrameAwareState, FrameId};

use nalgebra::Isometry3;
use std::sync::Arc;

pub(crate) struct MockOracleEstimatorNode {
    name: String,
    agent: AgentId,
    output: ChannelKey,
    descriptor: PortDescriptor,
    /// The composed kinematic carrier schema, cached at construction and shared into
    /// every published state by `Arc` clone. Composing it allocates and the
    /// shape never changes for the node's lifetime, so building it once and
    /// re-pointing each tick's state at it avoids rebuilding at 200 Hz × N agents.
    schema: Arc<StateSchema>,
}

impl MockOracleEstimatorNode {
    pub(crate) fn new(name: impl Into<String>, agent: AgentId) -> Self {
        let name = name.into();
        let output = estimator_output(&name);
        let descriptor = MockNodePortDescriptor::new()
            .input_oracle(oracle_pose_channel())
            .optional_oracle(oracle_twist_channel())
            .output_internal(output.clone())
            .build();
        Self {
            name,
            output: output.into(),
            schema: Arc::new(kinematic_carrier_schema(agent.clone())),
            agent,
            descriptor,
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

    fn execute(&self, bus: &PortBus, _tf: &dyn TfProvider, tick: TickContext) {
        // Cold-start: no oracle/pose yet → skip. The output slot keeps its
        // previous value under last-known-good semantics, or stays empty
        // if this is the first tick.
        let Some(pose_stamped) = bus.read::<Isometry3<f64>>(oracle_pose_channel().into()) else {
            return;
        };

        let twist = bus
            .read::<Twist>(oracle_twist_channel().into())
            .map(|s| s.value.clone());

        let mut state = FrameAwareState::from_schema(self.schema.clone(), pose_stamped.timestamp);

        // The carrier schema is fixed at construction, so a refused write is a
        // mismatch between that schema and these writers. Publishing the state
        // anyway would hand downstream a partly seeded estimate, so it is dropped.
        let written =
            write_pose_into(&mut state, &pose_stamped.value, &self.agent).and_then(|()| {
                match &twist {
                    // oracle/twist and the odom velocity blocks are both ENU —
                    // straight passthrough of linear and angular velocity.
                    Some(t) => write_world_twist_into(&mut state, t, &self.agent),
                    None => Ok(()),
                }
            });
        if let Err(e) = written {
            tracing::warn!(node = %self.name, "mock oracle estimator cannot write its state: {e}");
            return;
        }

        let stamped = Stamped {
            value: state,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };

        if let Err(ChannelError::UnknownChannel) = bus.write(self.output.clone(), stamped) {
            tracing::warn!(
                channel = %self.output,
                "mock oracle estimator state output channel is not wired into the DAG"
            );
        }
    }
}

fn write_pose_into(
    state: &mut FrameAwareState,
    pose: &Isometry3<f64>,
    agent: &AgentId,
) -> Result<(), BlockWriteError> {
    // Position and body→odom attitude in the estimate's odom frame. The oracle is
    // a perfect estimator: it reports truth, but publishes it into the same
    // odom-frame estimate slot the real filter fills, so downstream consumers
    // read one frame.
    state.set_pose(
        FrameId::base_link(agent.clone()),
        FrameId::odom(agent.clone()),
        Transform::<Flu, Enu>::from_isometry(*pose),
    )
}

/// Writes the twist into the estimate's velocity blocks: linear into
/// `Velocity(Odom)` and angular into `AngularVelocity(Odom)`.
///
/// The oracle reports both linear and angular velocity in the ENU frame, and the
/// odom-frame estimate is ENU-aligned, so `twist.angular` passes straight through
/// rather than being dropped.
fn write_world_twist_into(
    state: &mut FrameAwareState,
    twist: &Twist,
    agent: &AgentId,
) -> Result<(), BlockWriteError> {
    let odom = FrameId::odom(agent.clone());

    state.set_velocity(odom.clone(), FreeVector::<Enu>::from_raw(twist.linear))?;
    state.set_angular_velocity(odom, FreeVector::<Enu>::from_raw(twist.angular))
}

#[cfg(test)]
mod tests {
    //! Tests for [`MockOracleEstimatorNode`]:
    //! 1. Descriptor shape — oracle inputs + FrameAwareState output.
    //! 2. Execute round-trip — pose + twist published correctly.
    //! 3. Cold-start — no oracle → no publish.
    //! 4. Body-capability gate — build fails without `oracle/pose`,
    //!    succeeds with it. Locks the headline guarantee:
    //!    an oracle node cannot run on a body without truth.

    use super::*;
    use crate::body::{BodyCapabilities, Provenance, PublishedChannel};
    use crate::pipeline::PipelineBuildError;
    use crate::pipeline::PipelineBuilder;
    use crate::port::{ChannelKey, PortDescriptor};
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::transforms::{Convention, ErasedTransform};

    use helios_core::spatial::primitives::MonotonicTime;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    // --- Test fixtures ---

    struct MockRuntime;

    impl TfProvider for MockRuntime {
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
    }

    fn tick_at(now: f64, dt: f64) -> TickContext<'static> {
        TickContext::detached(MonotonicTime(now), dt, 0)
    }

    fn state_channel() -> ChannelKey {
        estimator_output("mock").into()
    }

    /// A descriptor that "produces" the oracle channels, so the bus has
    /// slots for them. The mock node depends on these slots existing —
    /// without a producer the bus skips the slot allocation and the
    /// test would silently no-op.
    fn oracle_producer_descriptor() -> PortDescriptor {
        PortDescriptor::new(
            vec![],
            vec![],
            vec![oracle_pose_channel().into(), oracle_twist_channel().into()],
            None,
        )
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
                    key: oracle_pose_channel().into(),
                    provenance: Provenance::Exact,
                },
                PublishedChannel {
                    key: oracle_twist_channel().into(),
                    provenance: Provenance::Exact,
                },
            ],
            ..Default::default()
        }
    }

    // --- Descriptor shape ---

    #[test]
    fn descriptor_requires_oracle_pose() {
        let node = MockOracleEstimatorNode::new("mock", AgentId::new("test_agent"));
        assert!(
            node.port_descriptor()
                .required_inputs()
                .any(|k| *k == ChannelKey::from(oracle_pose_channel())),
            "oracle/pose must be a required input"
        );
    }

    #[test]
    fn descriptor_marks_oracle_twist_optional() {
        let node = MockOracleEstimatorNode::new("mock", AgentId::new("test_agent"));
        assert!(
            node.port_descriptor()
                .optional_inputs()
                .any(|k| *k == ChannelKey::from(oracle_twist_channel())),
            "oracle/twist must be optional, not required"
        );
    }

    #[test]
    fn descriptor_outputs_frame_aware_state() {
        let node = MockOracleEstimatorNode::new("mock", AgentId::new("test_agent"));
        assert_eq!(node.port_descriptor().outputs(), vec![state_channel()]);
    }

    // --- Execute behavior ---

    #[test]
    fn republishes_pose_as_frame_aware_state() {
        let agent = AgentId::new("test_agent");
        let node = MockOracleEstimatorNode::new("mock", agent.clone());
        let bus = make_bus_with_oracle_producer(&node);

        // Write a known pose to oracle/pose.
        let pose = Isometry3::from_parts(
            Translation3::new(1.5, -2.0, 0.5),
            UnitQuaternion::identity(),
        );
        bus.write(
            oracle_pose_channel().into(),
            Stamped {
                value: pose,
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 99,
            },
        )
        .expect("oracle/pose slot must exist");

        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        let published = bus
            .read::<FrameAwareState>(state_channel())
            .expect("node must publish FrameAwareState");
        let recovered = published
            .value
            .pose::<Flu, Enu>(
                FrameId::base_link(agent.clone()),
                FrameId::odom(agent.clone()),
            )
            .expect("standard schema includes pose")
            .into_inner();
        let dx = (recovered.translation.vector - pose.translation.vector).norm();
        assert!(
            dx < 1e-9,
            "pose translation should round-trip exactly, dx={dx}"
        );
    }

    #[test]
    fn republishes_world_twist_into_state_world_slots() {
        // oracle/twist is already ENU; mock passes both linear and angular
        // velocity straight through into the estimate's odom-frame blocks.
        let agent = AgentId::new("test_agent");
        let node = MockOracleEstimatorNode::new("mock", agent.clone());
        let bus = make_bus_with_oracle_producer(&node);

        bus.write(
            oracle_pose_channel().into(),
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
            angular: Vector3::new(0.0, 0.0, 0.3),
        };
        bus.write(
            oracle_twist_channel().into(),
            Stamped {
                value: twist.clone(),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 99,
            },
        )
        .expect("oracle/twist slot must exist");

        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        let out = bus
            .read::<FrameAwareState>(state_channel())
            .expect("must publish");

        // Read back by typed block extractor — no index or layout ordering assumed.
        let v = out
            .value
            .velocity::<Enu>(FrameId::odom(agent.clone()))
            .expect("carrier has an odom linear-velocity block");
        assert!((v.x() - 2.0).abs() < 1e-9);
        assert!((v.y() - -1.0).abs() < 1e-9);
        assert!((v.z() - 0.5).abs() < 1e-9);

        let w = out
            .value
            .angular_velocity::<Enu>(FrameId::odom(agent.clone()))
            .expect("carrier has an odom angular-velocity block");
        assert!((w.z() - 0.3).abs() < 1e-9);
    }

    #[test]
    fn skips_publish_when_oracle_pose_absent() {
        let node = MockOracleEstimatorNode::new("mock", AgentId::new("test_agent"));
        let bus = make_bus_with_oracle_producer(&node);

        // No write to oracle/pose. Cold start.
        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        assert!(
            bus.read::<FrameAwareState>(state_channel()).is_none(),
            "must not publish without oracle/pose"
        );
    }

    #[test]
    fn stamp_uses_tick_now_and_node_id() {
        let node = MockOracleEstimatorNode::new("mock", AgentId::new("test_agent"));
        let bus = make_bus_with_oracle_producer(&node);
        bus.write(
            oracle_pose_channel().into(),
            Stamped {
                value: Isometry3::<f64>::identity(),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 99,
            },
        )
        .unwrap();

        let tick = TickContext::detached(MonotonicTime(3.5), 0.01, 7);
        node.execute(&bus, &MockRuntime, tick);

        let out = bus.read::<FrameAwareState>(state_channel()).unwrap();
        assert!((out.timestamp.0 - 3.5).abs() < 1e-9);
        assert_eq!(out.producer, 7);
    }

    // --- Build-time body-capability gate ---

    #[test]
    fn build_fails_when_body_lacks_oracle_pose() {
        let node = MockOracleEstimatorNode::new("mock", AgentId::new("test_agent"));
        let empty_body = BodyCapabilities {
            name: "no_oracle_body".to_string(),
            publishes: vec![],
            ..Default::default()
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
                    if body == "no_oracle_body" && *channel_key == ChannelKey::from(oracle_pose_channel())
            )
        });
        assert!(
            saw,
            "expected UnsatisfiedBodyCapabilities for oracle/pose, got: {errs:?}"
        );
    }

    #[test]
    fn build_succeeds_when_body_publishes_oracle_pose() {
        let node = MockOracleEstimatorNode::new("mock", AgentId::new("test_agent"));

        let result = PipelineBuilder::new()
            .add_node(Box::new(node))
            .with_body_capabilities(body_with_oracle())
            .build()
            .map(|_| ());

        assert!(result.is_ok(), "build should succeed: {result:?}");
    }
}
