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
//!    `Stamped<TrajectoryPoint>`. All other variants leave the bus untouched
//!    so the controller can fall back to last-known-good — matching the
//!    no-op-on-hold convention of [`crate::pipeline::nodes::SearchPlannerNode`].
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

use helios_core::data::primitives::TrajectoryPoint;
use helios_core::path_following::{PathFollower, PathFollowerResult};
use helios_core::planning::types::Path;

use crate::pipeline::builders::path_follower::PathFollowerInputBuilder;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, PortBus, PortDescriptor};
use crate::runtime::AgentRuntime;
use crate::stamped::{Health, Stamped};

/// Mutable per-tick state: the follower itself and the bus timestamp of the
/// most-recently-applied path. Held behind one [`Mutex`] so the two fields are
/// always updated together.
struct FollowerState {
    follower: Box<dyn PathFollower>,
    /// Bus [`Stamped::timestamp`] of the last `Path` we called `set_path` on.
    /// `None` = no path has ever been applied (cold start).
    last_path_timestamp: Option<f64>,
}

/// Pipeline node wrapping any [`PathFollower`] implementation.
///
/// The port descriptor adds `path_channel` to whatever the input builder
/// requires, declares `TrajectoryPoint @ ""` as its single output, and uses
/// `rate: None` — the follower fires every tick (controller rate).
pub struct PathFollowerNode {
    name: String,
    state: Mutex<FollowerState>,
    input_builder: Box<dyn PathFollowerInputBuilder>,
    path_channel: ChannelKey,
    descriptor: PortDescriptor,
}

impl PathFollowerNode {
    /// Build a node from a follower, an input builder, and the bus channel the
    /// upstream planner publishes [`Path`] on.
    ///
    /// `required_inputs` = builder requirements ∪ `[path_channel]`.
    /// `optional_inputs` mirrors the builder.
    /// `outputs` = `[TrajectoryPoint @ ""]`.
    /// `rate` = `None`.
    pub fn new(
        name: impl Into<String>,
        follower: Box<dyn PathFollower>,
        input_builder: Box<dyn PathFollowerInputBuilder>,
        path_channel: ChannelKey,
    ) -> Self {
        let mut required_inputs = input_builder.required_channels().to_vec();
        // Avoid silently double-declaring if a future builder ever includes the
        // path channel itself.
        if !required_inputs.contains(&path_channel) {
            required_inputs.push(path_channel.clone());
        }
        let optional_inputs = input_builder.optional_channels().to_vec();
        let descriptor = PortDescriptor {
            required_inputs,
            optional_inputs,
            outputs: vec![ChannelKey::of::<TrajectoryPoint>()],
            rate: None,
        };
        Self {
            name: name.into(),
            state: Mutex::new(FollowerState {
                follower,
                last_path_timestamp: None,
            }),
            input_builder,
            path_channel,
            descriptor,
        }
    }
}

impl PipelineNode for PathFollowerNode {
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

        // 4. Publish only on Active. All other variants (GoalReached, NoPath,
        //    Error) leave the bus untouched so the controller falls back to
        //    last-known-good — same convention as SearchPlannerNode.
        let trajectory_point = match result {
            PathFollowerResult::Active(tp) => tp,
            _ => return,
        };

        let stamped = Stamped {
            value: trajectory_point,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(ChannelKey::of::<TrajectoryPoint>(), stamped);
    }
}

#[cfg(test)]
mod tests {
    //! Wiring tests for [`PathFollowerNode`] — concrete follower behaviour
    //! (lookahead geometry, PID gains, goal-radius checks) is covered in
    //! `helios_core/src/path_following/`. Here we verify that `execute()`:
    //!   - calls `set_path` exactly once per distinct bus-timestamp on `path_channel`
    //!   - never calls `set_path` when the path timestamp is unchanged
    //!   - publishes a `Stamped<TrajectoryPoint>` only on `Active`
    //!   - is a no-op on `NoPath` / `GoalReached` / `Error`
    //!   - early-returns when the input builder yields `None`
    //!   - declares `path_channel` in its required inputs

    use super::*;

    use helios_core::data::primitives::{FrameHandle, MonotonicTime, TrajectoryPoint};
    use helios_core::frames::{FrameId, RobotState, StateVariable};
    use helios_core::path_following::{PathFollower, PathFollowerInputs, PathFollowerResult};
    use helios_core::planning::types::Path;

    use nalgebra::Isometry3;
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

    // --- Mock PathFollower with call recording ---

    #[derive(Default)]
    struct FollowerCalls {
        set_path_calls: u32,
        compute_calls: u32,
        last_dt: f64,
    }

    struct ScriptedFollower {
        calls: Arc<StdMutex<FollowerCalls>>,
        result: StdMutex<PathFollowerResult>,
    }

    impl ScriptedFollower {
        fn new(result: PathFollowerResult) -> (Self, Arc<StdMutex<FollowerCalls>>) {
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
        fn compute(&mut self, dt: f64, _inputs: &PathFollowerInputs) -> PathFollowerResult {
            let mut c = self.calls.lock().unwrap();
            c.compute_calls += 1;
            c.last_dt = dt;
            // clone the scripted result each call
            match &*self.result.lock().unwrap() {
                PathFollowerResult::Active(tp) => PathFollowerResult::Active(tp.clone()),
                PathFollowerResult::GoalReached => PathFollowerResult::GoalReached,
                PathFollowerResult::NoPath => PathFollowerResult::NoPath,
                PathFollowerResult::Error(s) => PathFollowerResult::Error(s.clone()),
            }
        }
        fn set_path(&mut self, _path: Path) {
            self.calls.lock().unwrap().set_path_calls += 1;
        }
        fn get_lookahead_waypoint(&self) -> Option<&TrajectoryPoint> {
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
                state: RobotState::new(vec![StateVariable::Px(FrameId::World)], 0.0),
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

    fn path_channel() -> ChannelKey {
        ChannelKey::named::<Path>("raw")
    }

    fn out_channel() -> ChannelKey {
        ChannelKey::of::<TrajectoryPoint>()
    }

    fn make_bus() -> PortBus {
        // Two descriptors: one that "produces" the path channel (so the bus
        // allocates a slot for it), and one that outputs the TrajectoryPoint
        // the node under test will publish.
        let path_producer = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![path_channel()],
            rate: None,
        };
        let traj_producer = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![out_channel()],
            rate: None,
        };
        PortBus::new(&[path_producer, traj_producer], vec![])
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 13,
        }
    }

    fn dummy_waypoint() -> TrajectoryPoint {
        TrajectoryPoint {
            state: RobotState::new(vec![StateVariable::Px(FrameId::World)], 0.0),
            state_dot: None,
            time: 0.0,
        }
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
        bus.write(path_channel(), stamped).unwrap();
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
        );
        assert_eq!(node.port_descriptor().outputs, vec![out_channel()]);
        assert!(node.port_descriptor().rate.is_none());
        assert!(node
            .port_descriptor()
            .required_inputs
            .contains(&path_channel()));
    }

    #[test]
    fn execute_publishes_trajectory_point_on_active() {
        let (follower, _calls) =
            ScriptedFollower::new(PathFollowerResult::Active(dummy_waypoint()));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);

        node.execute(&bus, &MockRuntime, tick_at(2.5, 0.05));

        let published = bus
            .read::<TrajectoryPoint>(out_channel())
            .expect("node must publish TrajectoryPoint on Active");
        assert!((published.timestamp.0 - 2.5).abs() < 1e-9);
        assert_eq!(published.producer, 13);
    }

    #[test]
    fn execute_no_op_on_goal_reached() {
        let (follower, _calls) = ScriptedFollower::new(PathFollowerResult::GoalReached);
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<TrajectoryPoint>(out_channel()).is_none());
    }

    #[test]
    fn execute_no_op_on_no_path() {
        let (follower, _calls) = ScriptedFollower::new(PathFollowerResult::NoPath);
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
        );
        let bus = make_bus();
        // No path published — follower still gets called (state was ready)
        // and reports NoPath; bus should remain empty.
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<TrajectoryPoint>(out_channel()).is_none());
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
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<TrajectoryPoint>(out_channel()).is_none());
    }

    #[test]
    fn execute_early_returns_when_input_builder_returns_none() {
        // Cold-start: state missing → follower never called, nothing published.
        let (follower, calls) =
            ScriptedFollower::new(PathFollowerResult::Active(dummy_waypoint()));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(NeverReadyBuilder {
                required: vec![],
                optional: vec![],
            }),
            path_channel(),
        );
        let bus = make_bus();
        publish_path(&bus, 1.0);
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<TrajectoryPoint>(out_channel()).is_none());
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
            ScriptedFollower::new(PathFollowerResult::Active(dummy_waypoint()));
        let node = PathFollowerNode::new(
            "pure_pursuit",
            Box::new(follower),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
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
