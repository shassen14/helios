//! [`PathFollowerNode`] — pipeline adapter for any [`PathFollower`]
//! implementation (Pure Pursuit, Stanley, SteeringPid).
//!
//! One node type for the whole path-following family today. The trait is small
//! enough (`compute` + `set_path` + lookahead/reset accessors) that there is no
//! family split.
//!
//! ## Execution skeleton
//!
//! 1. **Detect new path.** Read `Stamped<Path>` from `path_channel`. If the bus
//!    timestamp differs from the last-seen one, call [`PathFollower::set_path`]
//!    and remember the new timestamp.
//! 2. **Assemble inputs.** Ask the [`PathFollowerInputBuilder`] for
//!    [`PathFollowerInputs`] (state from the bus). `None` ⇒ cold-start, skip.
//! 3. **Compute.** Run [`PathFollower::compute`] with `dt` and the inputs.
//! 4. **Publish.** Only [`PathFollowerResult::Active`] writes a
//!    `Stamped<R>` (the follower's reference type). All other variants leave the bus untouched
//!    so the controller can fall back to last-known-good — matching the
//!    no-op-on-hold convention of `SearchPlannerNode`.
//!
//! ## Path-version tracking
//!
//! Detection uses the bus [`Stamped::timestamp`] (set by the upstream planner
//! node when it called `bus.write`) rather than `Path::timestamp` (set inside
//! the planner). The bus stamp is monotonic per writer at tick resolution and
//! is the right "did the bus slot get a new value?" signal.
//!
//! ## State layout
//!
//! Both the follower and the last-seen-path timestamp live behind one
//! [`Mutex`] (`Mutex<FollowerState>`). They are read and written together every
//! tick — a single lock makes that explicit and avoids two acquisitions.

use std::sync::Mutex;

use helios_core::control::ControlReference;
use helios_core::path_following::{PathFollower, PathFollowerResult};
use helios_core::planning::types::Path;

use super::input::PathFollowerInputBuilder;
use crate::pipeline::descriptor::AlgorithmNodePortDescriptor;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, InternalChannel, PortBus, PortDescriptor};
use crate::runtime::AgentRuntime;
use crate::stamped::{Health, Stamped};

/// Mutable per-tick state: the follower itself and the bus timestamp of the
/// most-recently-applied path. Held behind one [`Mutex`] so the two fields are
/// always updated together.
struct FollowerState<R: ControlReference> {
    follower: Box<dyn PathFollower<Reference = R>>,
    /// Bus [`Stamped::timestamp`] of the last `Path` we called `set_path` on.
    /// `None` = no path has ever been applied (cold start).
    last_path_timestamp: Option<f64>,
}

/// Pipeline node wrapping any [`PathFollower`] implementation.
///
/// The port descriptor adds `path_channel` to whatever the input builder
/// requires, declares the reference type `R` on its caller-supplied output
/// channel as its single output, and uses `rate: None` — the follower fires
/// every tick (controller rate).
pub(crate) struct PathFollowerNode<R: ControlReference> {
    name: String,
    state: Mutex<FollowerState<R>>,
    input_builder: Box<dyn PathFollowerInputBuilder>,
    path_channel: ChannelKey,
    output_channel: ChannelKey,
    descriptor: PortDescriptor,
}

impl<R: ControlReference> PathFollowerNode<R> {
    /// Build a node from a follower, an input builder, the bus channel the
    /// upstream planner publishes [`Path`] on, and the reference channel the
    /// follower publishes its setpoint on.
    ///
    /// `required_inputs` = builder requirements ∪ `[path_channel]`.
    /// `optional_inputs` mirrors the builder.
    /// `outputs` = `[output_channel]`.
    /// `rate` = `None`.
    pub(crate) fn new(
        name: impl Into<String>,
        follower: Box<dyn PathFollower<Reference = R>>,
        input_builder: Box<dyn PathFollowerInputBuilder>,
        path_channel: InternalChannel,
        output_channel: InternalChannel,
    ) -> Self {
        let path_channel_key: ChannelKey = path_channel.clone().into();
        let output_channel_key: ChannelKey = output_channel.clone().into();

        // Avoid silently double-declaring if a future builder ever includes
        // the path channel itself.
        let builder_required = input_builder.required_channels();
        let mut builder = AlgorithmNodePortDescriptor::new()
            .inputs_from_slices(builder_required, input_builder.optional_channels())
            .output_internal(output_channel);
        if !builder_required.contains(&path_channel_key) {
            builder = builder.input_internal(path_channel.clone());
        }
        let descriptor = builder.build();
        Self {
            name: name.into(),
            state: Mutex::new(FollowerState {
                follower,
                last_path_timestamp: None,
            }),
            input_builder,
            path_channel: path_channel_key,
            output_channel: output_channel_key,
            descriptor,
        }
    }
}

impl<R: ControlReference> PipelineNode for PathFollowerNode<R> {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, runtime: &dyn AgentRuntime, tick: TickContext) {
        // Skip the tick on a poisoned mutex rather than propagating the panic.
        let Ok(mut state) = self.state.lock() else {
            return;
        };

        // 1. New path? Compare against the bus Stamped.timestamp.
        if let Some(stamped_path) = bus.read::<Path>(self.path_channel.clone()) {
            let bus_ts = stamped_path.timestamp.0;
            let is_new = match state.last_path_timestamp {
                None => true,
                Some(prev) => bus_ts > prev,
            };
            if is_new {
                state.follower.set_path(stamped_path.value.clone());
                state.last_path_timestamp = Some(bus_ts);
            }
        }

        // 2. Assemble bus-sourced inputs (state).
        let Some(inputs) = self.input_builder.assemble(bus, runtime, &tick) else {
            return;
        };

        // 3. Compute the reference.
        let result = state.follower.compute(tick.dt, &inputs);

        // 4. Publish the reference on Active and on GoalReached. GoalReached
        //    carries a terminal "park here" setpoint from the follower, so it
        //    must reach the bus for the vehicle to stop at the goal rather than
        //    coast on its last command. NoPath/Error leave the bus untouched so
        //    the controller falls back to last-known-good — the transient-hold
        //    convention shared with SearchPlannerNode.
        let reference = match result {
            PathFollowerResult::Active(reference) | PathFollowerResult::GoalReached(reference) => {
                reference
            }
            PathFollowerResult::NoPath | PathFollowerResult::Error(_) => return,
        };

        let stamped = Stamped {
            value: reference,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(self.output_channel.clone(), stamped);
    }
}

#[cfg(test)]
mod tests {
    //! Wiring tests for [`PathFollowerNode`] — concrete follower behaviour
    //! (lookahead geometry, PID gains, goal-radius checks) is covered in
    //! `helios_core/src/path_following/`. Here we verify that `execute()`:
    //!   - calls `set_path` exactly once per distinct bus-timestamp on `path_channel`
    //!   - never calls `set_path` when the path timestamp is unchanged
    //!   - publishes a `Stamped<BodyTwistRef>` only on `Active`
    //!   - publishes the carried stop reference on `GoalReached`
    //!   - is a no-op on `NoPath` / `Error`
    //!   - early-returns when the input builder yields `None`
    //!   - declares `path_channel` in its required inputs

    use super::*;
    use helios_core::frames::transforms::{Convention, ErasedTransform};

    use helios_core::control::commands::BodyTwist;
    use helios_core::control::BodyTwistRef;
    use helios_core::data::primitives::{FrameHandle, MonotonicTime};
    use helios_core::estimation::carrier::kinematic_carrier_schema;
    use helios_core::frames::conventions::Enu;
    use helios_core::frames::quantities::Point;
    use helios_core::frames::{FrameAwareState, FrameId};
    use helios_core::path_following::{PathFollower, PathFollowerInputs, PathFollowerResult};
    use helios_core::planning::types::Path;

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

    // --- Mock PathFollower with call recording ---

    #[derive(Default)]
    struct FollowerCalls {
        set_path_calls: u32,
        compute_calls: u32,
        last_dt: f64,
    }

    struct ScriptedFollower {
        calls: Arc<StdMutex<FollowerCalls>>,
        result: StdMutex<PathFollowerResult<BodyTwistRef>>,
    }

    impl ScriptedFollower {
        fn new(result: PathFollowerResult<BodyTwistRef>) -> (Self, Arc<StdMutex<FollowerCalls>>) {
            let calls = Arc::new(StdMutex::new(FollowerCalls::default()));
            (
                Self {
                    calls: Arc::clone(&calls),
                    result: StdMutex::new(result),
                },
                calls,
            )
        }
    }

    impl PathFollower for ScriptedFollower {
        type Reference = BodyTwistRef;

        fn compute(
            &mut self,
            dt: f64,
            _inputs: &PathFollowerInputs,
        ) -> PathFollowerResult<BodyTwistRef> {
            let mut c = self.calls.lock().unwrap();
            c.compute_calls += 1;
            c.last_dt = dt;
            // clone the scripted result each call
            match &*self.result.lock().unwrap() {
                PathFollowerResult::Active(tp) => PathFollowerResult::Active(tp.clone()),
                PathFollowerResult::GoalReached(tp) => PathFollowerResult::GoalReached(tp.clone()),
                PathFollowerResult::NoPath => PathFollowerResult::NoPath,
                PathFollowerResult::Error(s) => PathFollowerResult::Error(s.clone()),
            }
        }
        fn set_path(&mut self, _path: Path) {
            self.calls.lock().unwrap().set_path_calls += 1;
        }
        fn get_lookahead_waypoint(&self) -> Option<&Point<Enu>> {
            None
        }
        fn reset(&mut self) {}
    }

    // --- Mock input builders ---

    struct AlwaysReadyBuilder {
        required: Vec<ChannelKey>,
        optional: Vec<ChannelKey>,
    }
    impl AlwaysReadyBuilder {
        fn new() -> Self {
            Self {
                required: vec![],
                optional: vec![],
            }
        }
    }
    impl PathFollowerInputBuilder for AlwaysReadyBuilder {
        fn assemble(
            &self,
            _bus: &PortBus,
            _runtime: &dyn AgentRuntime,
            _tick: &TickContext,
        ) -> Option<PathFollowerInputs> {
            Some(PathFollowerInputs {
                // A placeholder kinematic state; this mock never reads its contents.
                state: FrameAwareState::from_schema(
                    std::sync::Arc::new(kinematic_carrier_schema(FrameHandle(0))),
                    0.0,
                ),
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
    impl PathFollowerInputBuilder for NeverReadyBuilder {
        fn assemble(
            &self,
            _bus: &PortBus,
            _runtime: &dyn AgentRuntime,
            _tick: &TickContext,
        ) -> Option<PathFollowerInputs> {
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

    fn path_channel() -> InternalChannel {
        InternalChannel::named::<Path>("raw")
    }

    fn path_channel_key() -> ChannelKey {
        path_channel().into()
    }

    fn out_channel_internal() -> InternalChannel {
        InternalChannel::of::<BodyTwistRef>()
    }

    fn out_channel() -> ChannelKey {
        out_channel_internal().into()
    }

    fn make_bus() -> PortBus {
        // Two descriptors: one that "produces" the path channel (so the bus
        // allocates a slot for it), and one that outputs the BodyTwistRef
        // the node under test will publish.
        let path_producer = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![path_channel_key()],
            rate: None,
        };
        let traj_producer = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![out_channel()],
            rate: None,
        };
        PortBus::new(&[path_producer, traj_producer])
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 13,
        }
    }

    /// A single geometric path waypoint (ENU position only).
    fn dummy_waypoint() -> Point<Enu> {
        Point::new(0.0, 0.0, 0.0)
    }

    /// The follower's *output* reference — a body-frame twist setpoint, unlike
    /// the geometric path waypoints above.
    fn dummy_reference() -> BodyTwistRef {
        BodyTwistRef::new(BodyTwist::zero())
    }

    fn dummy_path() -> Path {
        Path {
            waypoints: vec![dummy_waypoint()],
            timestamp: 0.0,
            level_key: "global".into(),
        }
    }

    /// Publish a `Path` on the bus with a chosen Stamped.timestamp so we can
    /// drive new-path detection.
    fn publish_path(bus: &PortBus, ts: f64) {
        let stamped = Stamped {
            value: dummy_path(),
            timestamp: MonotonicTime(ts),
            health: Health::Ok,
            producer: 99,
        };
        bus.write(path_channel_key(), stamped).unwrap();
    }

    // --- Tests ---

    #[test]
    fn descriptor_outputs_trajectory_point_and_requires_path_channel() {
        let (follower, _calls) = ScriptedFollower::new(PathFollowerResult::NoPath);
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        assert_eq!(node.port_descriptor().outputs, vec![out_channel()]);
        assert!(node.port_descriptor().rate.is_none());
        assert!(node
            .port_descriptor()
            .required_inputs
            .contains(&path_channel_key()));
    }

    #[test]
    fn execute_publishes_trajectory_point_on_active() {
        let (follower, _calls) =
            ScriptedFollower::new(PathFollowerResult::Active(dummy_reference()));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);

        node.execute(&bus, &MockRuntime, tick_at(2.5, 0.05));

        let published = bus
            .read::<BodyTwistRef>(out_channel())
            .expect("node must publish BodyTwistRef on Active");
        assert!((published.timestamp.0 - 2.5).abs() < 1e-9);
        assert_eq!(published.producer, 13);
    }

    #[test]
    fn execute_publishes_to_its_configured_output_channel_not_a_hardcoded_one() {
        // Regression: the node must write the reference to the *same* channel its
        // descriptor declares as output, which the assembler names (`reference`).
        // A write hardcoded to a different slot is invisible — the bus allocates
        // the declared slot, controllers read it and find it empty, and the
        // vehicle never moves even though every node builds and ticks cleanly.
        let named_output = InternalChannel::named::<BodyTwistRef>("reference");
        let named_key: ChannelKey = named_output.clone().into();

        let (follower, _calls) =
            ScriptedFollower::new(PathFollowerResult::Active(dummy_reference()));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            named_output,
        );

        // A bus carrying both the named slot the node is configured to write and
        // the old unnamed slot, so we can assert the write lands on the former and
        // never the latter.
        let named_producer = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![named_key.clone()],
            rate: None,
        };
        let unnamed_producer = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![out_channel()],
            rate: None,
        };
        let path_producer = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![path_channel_key()],
            rate: None,
        };
        let bus = PortBus::new(&[named_producer, unnamed_producer, path_producer]);
        publish_path(&bus, 1.0);

        node.execute(&bus, &MockRuntime, tick_at(2.5, 0.05));

        assert!(
            bus.read::<BodyTwistRef>(named_key).is_some(),
            "follower must publish to its configured (named) output channel"
        );
        assert!(
            bus.read::<BodyTwistRef>(out_channel()).is_none(),
            "nothing must reach the old hardcoded unnamed slot"
        );
    }

    #[test]
    fn execute_publishes_stop_on_goal_reached() {
        // The follower carries a "park here" reference (a real follower makes this
        // a zero body twist). The node must forward it — not hold last-known-good —
        // or the controller keeps tracking the last nonzero velocity and overshoots.
        let stop = BodyTwistRef::new(BodyTwist::zero());
        let (follower, _calls) = ScriptedFollower::new(PathFollowerResult::GoalReached(stop));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);

        node.execute(&bus, &MockRuntime, tick_at(2.5, 0.05));

        let published = bus
            .read::<BodyTwistRef>(out_channel())
            .expect("GoalReached must publish an explicit stop, not hold last-known-good");
        assert_eq!(*published.value.twist(), BodyTwist::zero());
        assert!((published.timestamp.0 - 2.5).abs() < 1e-9);
        assert_eq!(published.producer, 13);
    }

    #[test]
    fn execute_no_op_on_no_path() {
        let (follower, _calls) = ScriptedFollower::new(PathFollowerResult::NoPath);
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        // No path published — follower still gets called (state was ready)
        // and reports NoPath; bus should remain empty.
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<BodyTwistRef>(out_channel()).is_none());
    }

    #[test]
    fn execute_no_op_on_error() {
        let (follower, _calls) =
            ScriptedFollower::new(PathFollowerResult::Error("missing field".into()));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<BodyTwistRef>(out_channel()).is_none());
    }

    #[test]
    fn execute_early_returns_when_input_builder_returns_none() {
        // Cold-start: state missing → follower never called, nothing published.
        let (follower, calls) =
            ScriptedFollower::new(PathFollowerResult::Active(dummy_reference()));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(NeverReadyBuilder {
                required: vec![],
                optional: vec![],
            }),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<BodyTwistRef>(out_channel()).is_none());
        assert_eq!(calls.lock().unwrap().compute_calls, 0);
    }

    #[test]
    fn set_path_called_once_on_first_observation() {
        let (follower, calls) = ScriptedFollower::new(PathFollowerResult::NoPath);
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert_eq!(calls.lock().unwrap().set_path_calls, 1);
    }

    #[test]
    fn set_path_not_called_again_for_same_timestamp() {
        let (follower, calls) = ScriptedFollower::new(PathFollowerResult::NoPath);
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        node.execute(&bus, &MockRuntime, tick_at(1.1, 0.1));
        node.execute(&bus, &MockRuntime, tick_at(1.2, 0.1));
        // One set_path total despite three ticks reading the same path.
        assert_eq!(calls.lock().unwrap().set_path_calls, 1);
    }

    #[test]
    fn set_path_called_again_when_new_path_published() {
        let (follower, calls) = ScriptedFollower::new(PathFollowerResult::NoPath);
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        publish_path(&bus, 2.0);
        node.execute(&bus, &MockRuntime, tick_at(2.0, 0.1));
        assert_eq!(calls.lock().unwrap().set_path_calls, 2);
    }

    #[test]
    fn execute_forwards_dt_to_follower() {
        let (follower, calls) =
            ScriptedFollower::new(PathFollowerResult::Active(dummy_reference()));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
            out_channel_internal(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.02));
        node.execute(&bus, &MockRuntime, tick_at(1.02, 0.04));
        let calls = calls.lock().unwrap();
        assert_eq!(calls.compute_calls, 2);
        assert!((calls.last_dt - 0.04).abs() < 1e-9);
    }
}
