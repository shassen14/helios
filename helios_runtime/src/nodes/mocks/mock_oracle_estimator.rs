//! [`MockOracleEstimatorNode`] — passthrough "estimator" that republishes the
//! body's true pose (and velocity, when available) as a `FrameAwareState`.
//!
//! No filter math. Used to skip estimation entirely for controller / planner
//! bring-up, demos where estimation drift would distract, and unit tests of
//! downstream stages.
//!
//! ## Execution
//!
//! 1. Read the body's instance of `truth.tf`. If absent → cold-start, return
//!    (downstream sees the last-known-good `FrameAwareState` from the
//!    previous tick, if any).
//! 2. Read the body's instance of `truth.twist` (optional).
//! 3. Build a `FrameAwareState` on the kinematic carrier schema from the pose
//!    and twist — position, linear/angular velocity, and attitude. The twist
//!    arrives in the body's axes and is rotated into the root's by the pose.
//! 4. Publish on the channel named after the node, held in the root the stack
//!    gives it, which it defines. Like any estimator it publishes no TF edge;
//!    the estimate seam's relay does.
//!
//! The truth pose is relative to the world frame its instance names as its
//! parent, and the node writes it as the pose in its root, so its root sits
//! at that frame. The descriptor declares this as a pin through the body's
//! instance of `truth.tf`.
//!
//! ## Body capability requirement
//!
//! The descriptor declares `truth.tf` as a required oracle input and
//! `truth.twist` as an optional one, each read at its body's `base_link`
//! instance. A body that does not publish `truth.tf` fails the build with
//! [`PipelineBuildError::UnsatisfiedBodyCapabilities`], and one whose truth
//! channels don't declare that instance with
//! [`PipelineBuildError::UnsatisfiedInstance`]. This is the type-level fence
//! that keeps oracle-reading nodes off bodies without truth.
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
//!     crate::PipelineBuildError::UnsatisfiedBodyCapabilities
//! [`PipelineBuildError::UnsatisfiedInstance`]:
//!     crate::PipelineBuildError::UnsatisfiedInstance

use crate::channels::estimate::estimator_output;
use crate::channels::{truth_tf, truth_twist};
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{
    ChannelKey, InstanceKey, Pinned, PortBus, PortDescriptor, PortDescriptorBuilder,
};
use crate::stamped::{Health, Stamped};

use helios_core::estimation::carrier::kinematic_carrier_schema;
use helios_core::estimation::frames::EstimationFrames;
use helios_core::estimation::schema::StateSchema;
use helios_core::interchange::motion::StampedTwist;
use helios_core::prelude::TfProvider;
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::transforms::erased::FrameMismatch;
use helios_core::spatial::transforms::tf::stamped::StampedTransform;
use helios_core::spatial::transforms::Transform;
use helios_core::spatial::{BlockWriteError, FrameAwareState, FrameId};

use std::fmt::Display;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub(crate) struct MockOracleEstimatorNode {
    name: String,
    frames: EstimationFrames,
    output: ChannelKey,
    descriptor: PortDescriptor,
    /// The composed kinematic carrier schema, cached at construction and shared into
    /// every published state by `Arc` clone. Composing it allocates and the
    /// shape never changes for the node's lifetime, so building it once and
    /// re-pointing each tick's state at it avoids rebuilding at 200 Hz × N agents.
    schema: Arc<StateSchema>,
    /// The body's instance of `truth.tf`.
    pose_instance: InstanceKey,
    /// The body's instance of `truth.twist`.
    twist_instance: InstanceKey,
    /// Set once a tick's truth has been refused and warned about. A refusal
    /// repeats every tick, so only the first is warned.
    refusal_warned: AtomicBool,
}

impl MockOracleEstimatorNode {
    /// A node named `name` publishing truth as an estimate held in `frames`,
    /// whose root it defines.
    pub(crate) fn new(name: impl Into<String>, frames: EstimationFrames) -> Self {
        let name = name.into();
        let output = estimator_output(&name);
        let schema = Arc::new(kinematic_carrier_schema(&frames));
        let root_convention = schema
            .convention_of(&frames.root)
            .expect("the carrier schema holds its position in the root");
        let descriptor = PortDescriptorBuilder::mock()
            .input(truth_tf())
            .reads_instance::<StampedTransform>(&truth_tf(), &frames.body)
            .optional(truth_twist())
            .reads_instance::<StampedTwist<Flu>>(&truth_twist(), &frames.body)
            .output(output.clone())
            .defines_frame(frames.root.clone(), root_convention)
            .pins_by_instance::<StampedTransform>(
                frames.root.clone(),
                Pinned::Position,
                &truth_tf(),
                &frames.body,
            )
            .build();
        Self {
            name,
            output,
            schema,
            pose_instance: InstanceKey::of::<StampedTransform>(&frames.body),
            twist_instance: InstanceKey::of::<StampedTwist<Flu>>(&frames.body),
            frames,
            descriptor,
            refusal_warned: AtomicBool::new(false),
        }
    }

    /// Writes the true pose, and the true twist when there is one, into
    /// `state`.
    fn write_truth(
        &self,
        state: &mut FrameAwareState,
        pose: &StampedTransform,
        twist: Option<&StampedTwist<Flu>>,
    ) -> Result<(), TruthRefused> {
        let body_to_root = pose
            .transform
            .typed::<Flu, Enu>()
            .map_err(TruthRefused::PoseAxes)?;
        // Position and body→root attitude in the estimate's root. The oracle
        // is a perfect estimator: it reports truth, but publishes it into the
        // same root-frame estimate slot the real filter fills, so downstream
        // consumers read one frame.
        state
            .set_pose(
                self.frames.body.clone(),
                self.frames.root.clone(),
                body_to_root,
            )
            .map_err(TruthRefused::State)?;

        let Some(twist) = twist else {
            return Ok(());
        };
        // The root sits at the pose's parent, a world frame the build pinned
        // it to, so only a motion relative to that frame is the root's.
        if twist.relative_to != pose.parent {
            return Err(TruthRefused::TwistReference {
                relative_to: twist.relative_to.clone(),
                pinned: pose.parent.clone(),
            });
        }
        // Rotating by the pose is right only for a twist in the body's axes.
        if twist.expressed_in != self.frames.body {
            return Err(TruthRefused::TwistAxes {
                expressed_in: twist.expressed_in.clone(),
            });
        }
        write_twist_into(state, twist, body_to_root, &self.frames).map_err(TruthRefused::State)
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
        // Cold-start: no truth pose yet → skip. The output slot keeps its
        // previous value under last-known-good semantics, or stays empty
        // if this is the first tick.
        let Some(pose) = bus.read_instance::<StampedTransform>(truth_tf(), &self.pose_instance)
        else {
            return;
        };
        let twist = bus.read_instance::<StampedTwist<Flu>>(truth_twist(), &self.twist_instance);

        let mut state = FrameAwareState::from_schema(self.schema.clone(), pose.timestamp);

        // Publishing a state the truth could not fill would hand downstream a
        // partly seeded estimate, so it is dropped.
        if let Err(refused) = self.write_truth(
            &mut state,
            &pose.value,
            twist.as_deref().map(|stamped| &stamped.value),
        ) {
            if !self.refusal_warned.swap(true, Ordering::Relaxed) {
                tracing::warn!(
                    node = %self.name,
                    cause = %refused,
                    "mock oracle estimator cannot write truth into its state; later refusals are not warned"
                );
            }
            return;
        }

        let stamped = Stamped {
            value: state,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };

        if let Err(error) = bus.write(self.output.clone(), stamped) {
            tracing::warn!(
                channel = %self.output,
                error = %error,
                "mock oracle estimator state output channel write failed"
            );
        }
    }
}

/// Why one tick's truth could not become a state.
#[derive(Debug)]
enum TruthRefused {
    /// The true pose does not carry body FLU to root ENU axes.
    PoseAxes(FrameMismatch),
    /// The true twist is relative to a frame other than the one the root
    /// is pinned to.
    TwistReference {
        relative_to: FrameId,
        pinned: FrameId,
    },
    /// The true twist is expressed in a frame other than the body.
    TwistAxes { expressed_in: FrameId },
    /// The carrier schema refused a block: a mismatch between the schema
    /// fixed at construction and these writers.
    State(BlockWriteError),
}

impl Display for TruthRefused {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TruthRefused::PoseAxes(mismatch) => write!(f, "true pose: {mismatch}"),
            TruthRefused::TwistReference {
                relative_to,
                pinned,
            } => write!(
                f,
                "true twist is relative to {relative_to}, not {pinned}, where the root is pinned"
            ),
            TruthRefused::TwistAxes { expressed_in } => {
                write!(f, "true twist is expressed in {expressed_in}, not the body")
            }
            TruthRefused::State(error) => write!(f, "{error}"),
        }
    }
}

/// Writes the body's twist into the estimate's velocity blocks, rotated from
/// the body's axes into the root's: linear into `Velocity(root)` and angular
/// into `AngularVelocity(root)`.
fn write_twist_into(
    state: &mut FrameAwareState,
    twist: &StampedTwist<Flu>,
    body_to_root: Transform<Flu, Enu>,
    frames: &EstimationFrames,
) -> Result<(), BlockWriteError> {
    let root = frames.root.clone();

    state.set_velocity(root.clone(), twist.linear.reexpress(&body_to_root))?;
    state.set_angular_velocity(root, twist.angular.reexpress(&body_to_root))
}

#[cfg(test)]
mod tests {
    //! Tests for [`MockOracleEstimatorNode`]:
    //! 1. Descriptor shape — truth inputs + FrameAwareState output.
    //! 2. Execute — the body's pose and twist published, the twist rotated
    //!    into the root's axes, other instances ignored, refusals warned once.
    //! 3. Cold-start — no truth → no publish.
    //! 4. Body-capability gate — build fails without `truth.tf`,
    //!    succeeds with it. Locks the headline guarantee:
    //!    an oracle node cannot run on a body without truth.

    use super::*;
    use crate::body::{BodyCapabilities, BodyChannel};
    use crate::host::HostDeclaration;
    use crate::pipeline::PipelineBuildError;
    use crate::pipeline::PipelineBuilder;
    use crate::pipeline::Placement;
    use crate::port::Provenance;
    use crate::port::{ChannelKey, PortDescriptor};

    use helios_core::prelude::AgentId;
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::primitives::MonotonicTime;
    use helios_core::spatial::quantities::FreeVector;
    use helios_core::spatial::transforms::tf::stamped::FrameEdge;
    use helios_core::spatial::transforms::{Convention, EdgeEnds, ErasedTransform, Transform};
    use helios_core::spatial::FrameId;

    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::collections::HashMap;
    use std::f64::consts::FRAC_PI_2;

    // --- Test fixtures ---

    /// The host's tick rate in the build tests.
    const TICK_RATE_HZ: f64 = 100.0;

    struct MockRuntime;

    impl TfProvider for MockRuntime {
        fn get_transform(
            &self,
            _: FrameId,
            _: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            Some(ErasedTransform::erase(Transform::<Flu, Flu>::identity()))
        }
    }

    fn agent() -> AgentId {
        AgentId::new("test_agent")
    }

    fn other_agent() -> AgentId {
        AgentId::new("other_agent")
    }

    fn datum() -> FrameId {
        FrameId::global("datum")
    }

    fn mock() -> MockOracleEstimatorNode {
        MockOracleEstimatorNode::new("mock", EstimationFrames::odom(agent()))
    }

    fn tick_at(now: f64, dt: f64) -> TickContext<'static> {
        TickContext::detached(MonotonicTime(now), dt, 0)
    }

    fn state_channel() -> ChannelKey {
        estimator_output("mock")
    }

    fn stamped<T>(value: T) -> Stamped<T> {
        Stamped {
            value,
            timestamp: MonotonicTime(0.0),
            health: Health::Ok,
            producer: 99,
        }
    }

    /// A truth source writing both agents' `base_link` instances, so the bus
    /// has slots for them. The mock node depends on these slots existing —
    /// without a producer the bus skips the slot allocation and the
    /// test would silently no-op.
    fn truth_producer_descriptor() -> PortDescriptor {
        let bodies = || {
            [
                FrameId::base_link(agent()),
                FrameId::base_link(other_agent()),
            ]
        };
        PortDescriptorBuilder::mock()
            .output_keyed::<StampedTransform>(
                truth_tf(),
                bodies().map(|child| FrameEdge {
                    child,
                    parent: datum(),
                }),
            )
            .output_keyed::<StampedTwist<Flu>>(truth_twist(), bodies())
            .build()
    }

    fn make_bus_with_truth_producer(node: &MockOracleEstimatorNode) -> PortBus {
        let producer = truth_producer_descriptor();
        PortBus::of_descriptors([node.port_descriptor(), &producer])
    }

    /// The pose of `body` in the datum, tagged `from` → `to`.
    fn pose_tagged(
        body: FrameId,
        pose: Isometry3<f64>,
        from: Convention,
        to: Convention,
    ) -> StampedTransform {
        StampedTransform {
            parent: datum(),
            child: body,
            stamp: MonotonicTime(0.0),
            transform: ErasedTransform::new(
                pose,
                EdgeEnds {
                    child: from,
                    parent: to,
                },
            ),
        }
    }

    fn write_pose(bus: &PortBus, body: FrameId, pose: Isometry3<f64>) {
        bus.write_keyed(
            truth_tf(),
            stamped(pose_tagged(body, pose, Convention::Flu, Convention::Enu)),
        )
        .expect("truth.tf instance must exist");
    }

    /// `base_link`'s twist relative to the datum, expressed in `expressed_in`.
    fn twist_in(
        expressed_in: FrameId,
        linear: Vector3<f64>,
        angular: Vector3<f64>,
    ) -> StampedTwist<Flu> {
        StampedTwist {
            moving: FrameId::base_link(agent()),
            relative_to: datum(),
            expressed_in,
            stamp: MonotonicTime(0.0),
            linear: FreeVector::from_raw(linear),
            angular: FreeVector::from_raw(angular),
        }
    }

    fn write_twist(bus: &PortBus, twist: StampedTwist<Flu>) {
        bus.write_keyed(truth_twist(), stamped(twist))
            .expect("truth.twist instance must exist");
    }

    fn body_with_truth() -> BodyCapabilities {
        let body = FrameId::base_link(agent());
        BodyCapabilities {
            publishes: vec![
                BodyChannel::new(truth_tf(), "truth", Provenance::Exact)
                    .with_instances::<StampedTransform>([FrameEdge {
                        child: body.clone(),
                        parent: datum(),
                    }]),
                BodyChannel::new(truth_twist(), "truth", Provenance::Exact)
                    .with_instances::<StampedTwist<Flu>>([body])
                    .with_reference(datum()),
            ],
            ..Default::default()
        }
    }

    fn assert_near(actual: FreeVector<Enu>, expected: [f64; 3]) {
        let error = (actual.into_inner() - Vector3::from(expected)).norm();
        assert!(error < 1e-9, "{actual:?} is not {expected:?}");
    }

    // --- Descriptor shape ---

    #[test]
    fn descriptor_requires_truth_tf() {
        assert!(
            mock()
                .port_descriptor()
                .required_inputs()
                .any(|k| *k == truth_tf()),
            "truth.tf must be a required input"
        );
    }

    #[test]
    fn descriptor_marks_truth_twist_optional() {
        assert!(
            mock()
                .port_descriptor()
                .optional_inputs()
                .any(|k| *k == truth_twist()),
            "truth.twist must be optional, not required"
        );
    }

    /// Each truth channel is read at its body's instance and no other.
    #[test]
    fn descriptor_reads_its_bodys_instance_of_each_truth_channel() {
        let node = mock();
        let body = FrameId::base_link(agent());
        let read: Vec<(&ChannelKey, &[InstanceKey])> = node
            .port_descriptor()
            .inputs()
            .map(|input| (input.channel(), input.instances()))
            .collect();

        assert_eq!(
            read,
            [
                (
                    &truth_tf(),
                    &[InstanceKey::of::<StampedTransform>(&body)][..]
                ),
                (
                    &truth_twist(),
                    &[InstanceKey::of::<StampedTwist<Flu>>(&body)][..]
                ),
            ]
        );
    }

    #[test]
    fn descriptor_outputs_frame_aware_state() {
        assert_eq!(
            mock()
                .port_descriptor()
                .outputs()
                .cloned()
                .collect::<Vec<_>>(),
            vec![state_channel()]
        );
    }

    /// The root sits where the body's truth instance is expressed, so the
    /// node declares the pin through that instance.
    #[test]
    fn descriptor_pins_the_root_through_its_bodys_truth_instance() {
        let node = mock();
        let pins = node.port_descriptor().pins();

        assert_eq!(pins.len(), 1);
        assert_eq!(pins[0].frame(), &FrameId::odom(agent()));
        assert_eq!(pins[0].what(), Pinned::Position);
        assert_eq!(pins[0].channel(), &truth_tf());
        assert_eq!(
            pins[0].instance(),
            Some(&InstanceKey::of::<StampedTransform>(&FrameId::base_link(
                agent()
            )))
        );
    }

    // --- Execute behavior ---

    #[test]
    fn republishes_pose_as_frame_aware_state() {
        let node = mock();
        let bus = make_bus_with_truth_producer(&node);

        let pose = Isometry3::from_parts(
            Translation3::new(1.5, -2.0, 0.5),
            UnitQuaternion::identity(),
        );
        write_pose(&bus, FrameId::base_link(agent()), pose);

        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        let published = bus
            .read::<FrameAwareState>(state_channel())
            .expect("node must publish FrameAwareState");
        let recovered = published
            .value
            .pose::<Flu, Enu>(FrameId::base_link(agent()), FrameId::odom(agent()))
            .expect("standard schema includes pose")
            .into_inner();
        let dx = (recovered.translation.vector - pose.translation.vector).norm();
        assert!(
            dx < 1e-9,
            "pose translation should round-trip exactly, dx={dx}"
        );
    }

    /// Facing north (yawed a quarter turn left of east), the body's forward
    /// is the root's north and its left the root's west.
    #[test]
    fn rotates_the_body_twist_into_the_roots_axes() {
        let node = mock();
        let bus = make_bus_with_truth_producer(&node);

        let facing_north = Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2),
        );
        write_pose(&bus, FrameId::base_link(agent()), facing_north);
        write_twist(
            &bus,
            twist_in(
                FrameId::base_link(agent()),
                Vector3::new(2.0, -1.0, 0.5),
                Vector3::new(0.1, 0.0, 0.3),
            ),
        );

        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        let out = bus
            .read::<FrameAwareState>(state_channel())
            .expect("must publish");
        let root = FrameId::odom(agent());
        assert_near(
            out.value
                .velocity::<Enu>(root.clone())
                .expect("carrier has a root linear-velocity block"),
            [1.0, 2.0, 0.5],
        );
        assert_near(
            out.value
                .angular_velocity::<Enu>(root)
                .expect("carrier has a root angular-velocity block"),
            [0.0, 0.1, 0.3],
        );
    }

    /// The truth source publishes every agent's body; the mock reads its own.
    #[test]
    fn reads_only_its_own_bodys_instance() {
        let node = mock();
        let bus = make_bus_with_truth_producer(&node);

        write_pose(
            &bus,
            FrameId::base_link(other_agent()),
            Isometry3::identity(),
        );
        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        assert!(
            bus.read::<FrameAwareState>(state_channel()).is_none(),
            "another agent's truth is not this body's"
        );
    }

    #[test]
    fn skips_publish_when_truth_absent() {
        let node = mock();
        let bus = make_bus_with_truth_producer(&node);

        // No write to truth.tf. Cold start.
        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        assert!(
            bus.read::<FrameAwareState>(state_channel()).is_none(),
            "must not publish without truth.tf"
        );
    }

    /// A twist in a sensor's axes would be rotated as though it were the
    /// body's, so the tick is dropped and warned about once.
    #[test]
    fn a_twist_in_another_frames_axes_is_refused() {
        let node = mock();
        let bus = make_bus_with_truth_producer(&node);

        write_pose(&bus, FrameId::base_link(agent()), Isometry3::identity());
        write_twist(
            &bus,
            twist_in(
                FrameId::agent(agent(), "lidar"),
                Vector3::x(),
                Vector3::zeros(),
            ),
        );
        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        assert!(bus.read::<FrameAwareState>(state_channel()).is_none());
        assert!(node.refusal_warned.load(Ordering::Relaxed));
    }

    /// The root sits at the pose's parent, so a motion relative to any other
    /// frame is not the root's: the tick is dropped and warned about once.
    #[test]
    fn a_twist_relative_to_another_frame_is_refused() {
        let node = mock();
        let bus = make_bus_with_truth_producer(&node);

        write_pose(&bus, FrameId::base_link(agent()), Isometry3::identity());
        let mut twist = twist_in(FrameId::base_link(agent()), Vector3::x(), Vector3::zeros());
        twist.relative_to = FrameId::global("mocap");
        write_twist(&bus, twist);
        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        assert!(bus.read::<FrameAwareState>(state_channel()).is_none());
        assert!(node.refusal_warned.load(Ordering::Relaxed));
    }

    #[test]
    fn a_pose_in_other_axes_is_refused() {
        let node = mock();
        let bus = make_bus_with_truth_producer(&node);

        let flu_to_flu = pose_tagged(
            FrameId::base_link(agent()),
            Isometry3::identity(),
            Convention::Flu,
            Convention::Flu,
        );
        bus.write_keyed(truth_tf(), stamped(flu_to_flu))
            .expect("truth.tf instance must exist");
        node.execute(&bus, &MockRuntime, tick_at(0.0, 0.01));

        assert!(bus.read::<FrameAwareState>(state_channel()).is_none());
        assert!(node.refusal_warned.load(Ordering::Relaxed));
    }

    #[test]
    fn stamp_uses_tick_now_and_node_id() {
        let node = mock();
        let bus = make_bus_with_truth_producer(&node);
        write_pose(&bus, FrameId::base_link(agent()), Isometry3::identity());

        let tick = TickContext::detached(MonotonicTime(3.5), 0.01, 7);
        node.execute(&bus, &MockRuntime, tick);

        let out = bus
            .read::<FrameAwareState>(state_channel())
            .expect("must publish");
        assert!((out.timestamp.0 - 3.5).abs() < 1e-9);
        assert_eq!(out.producer, 7);
    }

    // --- Build-time body-capability gate ---

    #[test]
    fn build_fails_when_body_lacks_truth_tf() {
        let empty_body = BodyCapabilities {
            publishes: vec![],
            ..Default::default()
        };

        // .map(|_| ()) drops the AutonomyPipeline so expect_err / Debug
        // formatting work — AutonomyPipeline doesn't implement Debug.
        let errs = PipelineBuilder::new(agent())
            .add_node(Box::new(mock()))
            .with_body_capabilities(empty_body)
            .build()
            .map(|_| ())
            .expect_err("build must reject mock without truth.tf");

        let saw = errs.iter().any(|e| {
            matches!(
                e,
                PipelineBuildError::UnsatisfiedBodyCapabilities { channel_key, .. }
                    if *channel_key == truth_tf()
            )
        });
        assert!(
            saw,
            "expected UnsatisfiedBodyCapabilities for truth.tf, got: {errs:?}"
        );
    }

    /// Truth naming another frame than the agent's `base_link`, as mocap
    /// naming its rigid body would, is the body's gap at build, not a root
    /// left free and a mock that never publishes.
    #[test]
    fn build_fails_when_the_bodys_truth_names_another_frame() {
        let mocap_body = FrameId::global("mocap_body");
        let body = BodyCapabilities {
            publishes: vec![
                BodyChannel::new(truth_tf(), "truth", Provenance::Exact)
                    .with_instances::<StampedTransform>([FrameEdge {
                        child: mocap_body.clone(),
                        parent: datum(),
                    }]),
                BodyChannel::new(truth_twist(), "truth", Provenance::Exact)
                    .with_instances::<StampedTwist<Flu>>([mocap_body]),
            ],
            ..Default::default()
        };

        let errs = PipelineBuilder::new(agent())
            .add_node(Box::new(mock()))
            .with_body_capabilities(body)
            .with_host(HostDeclaration {
                tick_rate_hz: TICK_RATE_HZ,
                world_frames: HashMap::from([(datum(), Convention::Enu)]),
            })
            .build()
            .map(|_| ())
            .expect_err("build must reject truth without the body's instance");

        let refused: Vec<(&ChannelKey, &InstanceKey)> = errs
            .iter()
            .filter_map(|e| match e {
                PipelineBuildError::UnsatisfiedInstance {
                    channel, instance, ..
                } => Some((channel, instance)),
                _ => None,
            })
            .collect();
        let body = FrameId::base_link(agent());
        assert_eq!(
            refused,
            [
                (&truth_tf(), &InstanceKey::of::<StampedTransform>(&body)),
                (&truth_twist(), &InstanceKey::of::<StampedTwist<Flu>>(&body)),
            ],
            "{errs:?}"
        );
        assert_eq!(errs.len(), 2, "{errs:?}");
    }

    /// A twist relative to another world frame than the pose's parent can't
    /// be written as velocity in the root the pose places there, so it is
    /// refused at build, naming both inputs.
    #[test]
    fn build_fails_when_the_twist_is_relative_to_another_world_frame() {
        let site = FrameId::global("site");
        let mut body = body_with_truth();
        body.publishes[1] = body.publishes[1].clone().with_reference(site.clone());

        let errs = PipelineBuilder::new(agent())
            .add_node(Box::new(mock()))
            .with_body_capabilities(body)
            .with_host(HostDeclaration {
                tick_rate_hz: TICK_RATE_HZ,
                world_frames: HashMap::from([
                    (datum(), Convention::Enu),
                    (site.clone(), Convention::Enu),
                ]),
            })
            .build()
            .map(|_| ())
            .expect_err("build must reject a twist in another world frame");

        let [PipelineBuildError::ReferencesDisagree { references, .. }] = errs.as_slice() else {
            panic!("expected only disagreeing references, got {errs:?}");
        };
        assert_eq!(references, &[(truth_twist(), site), (truth_tf(), datum())]);
    }

    /// The host declares the datum the truth is published in, so the pin
    /// through it holds and anchors the root there.
    #[test]
    fn build_succeeds_when_body_publishes_truth_tf() {
        let result = PipelineBuilder::new(agent())
            .add_node(Box::new(mock()))
            .with_body_capabilities(body_with_truth())
            .with_host(HostDeclaration {
                tick_rate_hz: TICK_RATE_HZ,
                world_frames: HashMap::from([(datum(), Convention::Enu)]),
            })
            .build();

        let Ok(pipeline) = result else {
            panic!("build should succeed: {:?}", result.err());
        };
        let anchored: Vec<(&FrameId, &FrameId)> = pipeline
            .frame_placements()
            .iter()
            .filter_map(|placement| match placement.placement() {
                Placement::Anchored(edge) => Some((&edge.child, &edge.parent)),
                Placement::Free(_) => None,
            })
            .collect();
        assert_eq!(anchored, [(&FrameId::odom(agent()), &datum())]);
    }
}
