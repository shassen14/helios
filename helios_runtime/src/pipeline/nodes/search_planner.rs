//! [`SearchPlannerNode`] — pipeline adapter for any search-family planner
//! (A*, Dijkstra, D*).
//!
//! One node type per algorithm family.
//! Today the only impl is A*, but the node is generic over
//! [`SearchPlanner`] so Dijkstra/D* drop in without touching the pipeline.
//!
//! ## Rate ownership
//!
//! The planner owns its own replan decision via
//! [`SearchPlanner::should_replan`] — A*'s deviation-based replan cannot be
//! expressed by a fixed-Hz [`RateTimer`]. The node's `PortDescriptor::rate`
//! is therefore `None`: the node fires every tick, and `plan()` decides
//! whether to actually search or return `PathStillValid`.
//!
//! ## Bus output
//!
//! The output channel is supplied at construction (typically
//! `ChannelKey::named::<Path>("raw")` or `ChannelKey::of::<Path>()`). The
//! node writes a `Stamped<Path>` only on [`PlannerResult::Path`] or
//! [`PlannerResult::GoalOutsideMap`] — all other variants leave the bus
//! untouched, so the previously-cached path remains available via
//! last-known-good semantics.
//!
//! [`RateTimer`]: crate::pipeline::rate_gate::RateTimer

use std::sync::Mutex;

use helios_core::planning::types::PlannerResult;
use helios_core::planning::SearchPlanner;

use crate::pipeline::builders::planner::SearchPlannerInputBuilder;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, PortBus, PortDescriptor};
use crate::runtime::AgentRuntime;
use crate::stamped::{Health, Stamped};

/// Pipeline node wrapping any search-family planner.
///
/// Construction is via [`Self::new`]. The port descriptor is derived from the
/// input builder — required/optional channels come from there directly.
pub struct SearchPlannerNode {
    name: String,
    planner: Mutex<Box<dyn SearchPlanner>>,
    input_builder: Box<dyn SearchPlannerInputBuilder>,
    path_channel: ChannelKey,
    descriptor: PortDescriptor,
}

impl SearchPlannerNode {
    /// Build a node from a planner, an input builder, and the bus channel
    /// the produced [`Path`] should be published on.
    ///
    /// The descriptor's `outputs` is `[path_channel]`, `required_inputs` /
    /// `optional_inputs` mirror the input builder, and `rate` is `None`
    /// (planner owns the replan decision — see the module-level docs).
    pub fn new(
        name: impl Into<String>,
        planner: Box<dyn SearchPlanner>,
        input_builder: Box<dyn SearchPlannerInputBuilder>,
        path_channel: ChannelKey,
    ) -> Self {
        let required_inputs = input_builder.required_channels().to_vec();
        let optional_inputs = input_builder.optional_channels().to_vec();
        let descriptor = PortDescriptor {
            required_inputs,
            optional_inputs,
            outputs: vec![path_channel.clone()],
            rate: None,
        };
        Self {
            name: name.into(),
            planner: Mutex::new(planner),
            input_builder,
            path_channel,
            descriptor,
        }
    }
}

impl PipelineNode for SearchPlannerNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, runtime: &dyn AgentRuntime, tick: TickContext) {
        // Cold-start / sensor dropout: required inputs absent → skip this tick.
        let Some(inputs) = self.input_builder.assemble(bus, runtime, &tick) else {
            return;
        };

        // Skip the tick on a poisoned mutex rather than propagating the panic.
        let Ok(mut planner) = self.planner.lock() else {
            return;
        };

        let result = planner.plan(tick.now.0, &inputs);

        let path = match result {
            PlannerResult::Path(p) | PlannerResult::GoalOutsideMap(p) => p,
            // Hold / unreachable / error / goal-reached / no-goal: don't touch
            // the bus. The last successfully published path remains under
            // last-known-good semantics.
            _ => return,
        };

        let stamped = Stamped {
            value: path,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(self.path_channel.clone(), stamped);
    }
}

#[cfg(test)]
mod tests {
    //! Wiring tests for [`SearchPlannerNode`] — A*'s algorithmic behavior is
    //! covered in `helios_core/src/planning/astar/`. Here we verify that
    //! `execute()` correctly:
    //!   - early-returns when the input builder yields `None`
    //!   - writes a `Stamped<Path>` on `PlannerResult::Path`
    //!   - writes on `PlannerResult::GoalOutsideMap`
    //!   - is a no-op on `PlannerResult::PathStillValid` (and other Hold variants)

    use super::*;

    use helios_core::data::primitives::{FrameHandle, MonotonicTime, TrajectoryPoint};
    use helios_core::frames::{FrameId, RobotState, StateVariable};
    use helios_core::mapping::MapData;
    use helios_core::planning::types::Path;
    use helios_core::planning::SearchPlannerInputs;

    use nalgebra::{DMatrix, Isometry3};
    use std::sync::Mutex as StdMutex;

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

    // --- Mock SearchPlanner that returns a scripted result ---

    struct ScriptedPlanner {
        result: StdMutex<Option<PlannerResult>>,
        plan_calls: StdMutex<u32>,
    }

    impl ScriptedPlanner {
        fn new(result: PlannerResult) -> Self {
            Self {
                result: StdMutex::new(Some(result)),
                plan_calls: StdMutex::new(0),
            }
        }
    }

    impl SearchPlanner for ScriptedPlanner {
        fn plan(&mut self, _now: f64, _inputs: &SearchPlannerInputs) -> PlannerResult {
            *self.plan_calls.lock().unwrap() += 1;
            // take() the scripted result; subsequent calls fall back to Hold.
            self.result
                .lock()
                .unwrap()
                .take()
                .unwrap_or(PlannerResult::PathStillValid)
        }
        fn should_replan(&self, _now: f64, _inputs: &SearchPlannerInputs) -> bool {
            true
        }
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
    impl SearchPlannerInputBuilder for AlwaysReadyBuilder {
        fn assemble(
            &self,
            _bus: &PortBus,
            _runtime: &dyn AgentRuntime,
            _tick: &TickContext,
        ) -> Option<SearchPlannerInputs> {
            Some(SearchPlannerInputs {
                state: RobotState::new(vec![StateVariable::Px(FrameId::World)], 0.0),
                map: MapData::OccupancyGrid2D {
                    origin: Isometry3::identity(),
                    resolution: 1.0,
                    data: DMatrix::from_element(1, 1, 0u8),
                    version: 0,
                },
                goal: None,
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
    impl SearchPlannerInputBuilder for NeverReadyBuilder {
        fn assemble(
            &self,
            _bus: &PortBus,
            _runtime: &dyn AgentRuntime,
            _tick: &TickContext,
        ) -> Option<SearchPlannerInputs> {
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

    fn make_bus() -> PortBus {
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![path_channel()],
            rate: None,
        };
        PortBus::new(&[descriptor], vec![])
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 7,
        }
    }

    fn dummy_path() -> Path {
        let wp = TrajectoryPoint {
            state: RobotState::new(vec![StateVariable::Px(FrameId::World)], 0.0),
            state_dot: None,
            time: 0.0,
        };
        Path {
            waypoints: vec![wp],
            timestamp: 0.0,
            level_key: "global".into(),
        }
    }

    // --- Tests ---

    #[test]
    fn descriptor_outputs_configured_path_channel() {
        let node = SearchPlannerNode::new(
            "astar",
            Box::new(ScriptedPlanner::new(PlannerResult::PathStillValid)),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
        );
        assert_eq!(node.port_descriptor().outputs, vec![path_channel()]);
        assert!(node.port_descriptor().rate.is_none());
    }

    #[test]
    fn execute_writes_path_on_planner_result_path() {
        let node = SearchPlannerNode::new(
            "astar",
            Box::new(ScriptedPlanner::new(PlannerResult::Path(dummy_path()))),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
        );
        let bus = make_bus();
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));

        let published = bus
            .read::<Path>(path_channel())
            .expect("node must publish a Path on PlannerResult::Path");
        assert!((published.timestamp.0 - 1.0).abs() < 1e-9);
        assert_eq!(published.producer, 7);
    }

    #[test]
    fn execute_writes_path_on_goal_outside_map() {
        let node = SearchPlannerNode::new(
            "astar",
            Box::new(ScriptedPlanner::new(PlannerResult::GoalOutsideMap(
                dummy_path(),
            ))),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
        );
        let bus = make_bus();
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<Path>(path_channel()).is_some());
    }

    #[test]
    fn execute_no_op_on_path_still_valid() {
        let node = SearchPlannerNode::new(
            "astar",
            Box::new(ScriptedPlanner::new(PlannerResult::PathStillValid)),
            Box::new(AlwaysReadyBuilder::new()),
            path_channel(),
        );
        let bus = make_bus();
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<Path>(path_channel()).is_none());
    }

    #[test]
    fn execute_early_returns_when_input_builder_returns_none() {
        // Distinct from Gaussian node: with no inputs the planner cannot run,
        // and unlike the estimator there is no useful "prior" to publish.
        let planner = ScriptedPlanner::new(PlannerResult::Path(dummy_path()));
        let node = SearchPlannerNode::new(
            "astar",
            Box::new(planner),
            Box::new(NeverReadyBuilder {
                required: vec![],
                optional: vec![],
            }),
            path_channel(),
        );
        let bus = make_bus();
        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        assert!(bus.read::<Path>(path_channel()).is_none());
    }
}
