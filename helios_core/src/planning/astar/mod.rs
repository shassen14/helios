// helios_core/src/planning/astar/mod.rs

//! A\* path planner on [`MapData::OccupancyGrid2D`](crate::mapping::MapData) maps.
//!
//! # Module layout
//!
//! | Sub-module | Contents |
//! |---|---|
//! | `grid_space` | `OccupancyGridSpace` — coordinate conversion, free-cell query, neighbour enumeration, Bresenham LOS |
//! | `search` | `AStarSearchBuffers`, `AStarNode`, `octile_heuristic`, `run_astar` |
//! | `smoothing` | `smooth_path`, `make_waypoint` |
//!
//! External callers only need [`AStarConfig`] and [`AStarPlanner`]; everything
//! else is `pub(super)` and invisible outside the `astar` module.
//!
//! # Lifecycle
//!
//! ```text
//! AStarPlanner::new(config)
//!   └─ plan() called each tick by the autonomy pipeline
//!        ├─ arrival check  → GoalReached
//!        ├─ rate / deviation gate → PathStillValid
//!        ├─ run_astar(&mut self.buffers)  ← reuses pre-allocated heap
//!        ├─ cell path → world waypoints
//!        └─ optional smooth_path()
//! ```
//!
//! # Pre-allocation
//!
//! The `BinaryHeap` and two `HashMap`s used by A\* are owned by
//! [`AStarPlanner`] via `AStarSearchBuffers` and cleared (not reallocated)
//! at the start of each search.  This avoids repeated heap allocation at
//! planning frequency (typically 1–10 Hz on a 100×100 grid).

mod grid_space;
mod search;
mod smoothing;

use nalgebra::Vector2;

use crate::frames::{FrameId, StateVariable};
use crate::mapping::MapData;
use crate::planning::SearchPlannerInputs;

use super::search_space::SearchSpace;
use super::types::{Path, PlannerGoal, PlannerResult, PlannerStatus};
use super::SearchPlanner;

use grid_space::OccupancyGridSpace;
use search::{run_astar, AStarSearchBuffers};
use smoothing::{make_waypoint, smooth_path};

// =========================================================================
// == Config ==
// =========================================================================

/// Configuration for [`AStarPlanner`].
///
/// All values come from TOML; never hardcode them in Rust source.
pub struct AStarConfig {
    /// Planning frequency (Hz). The planner will not replan more often than
    /// `1 / rate_hz` seconds.  Set to `0.0` to disable the rate gate (replan
    /// every tick).
    pub rate_hz: f64,
    /// Distance (metres) from the goal at which the robot is considered to
    /// have arrived.  Checked before A\* runs.
    pub arrival_tolerance_m: f64,
    /// Cells with raw occupancy value `>= occupancy_threshold` are treated as
    /// obstacles.  Typical values: `1` (any mark = obstacle), `128` (ROS default).
    pub occupancy_threshold: u8,
    /// Maximum number of node expansions per search.  Prevents runaway
    /// computation on very large or pathological grids.
    pub max_search_depth: usize,
    /// If `true`, run the string-pull smoother on the raw A\* cell path to
    /// remove collinear intermediate waypoints.
    pub enable_path_smoothing: bool,
    /// If `true`, trigger a replan when the robot drifts more than
    /// `deviation_tolerance_m` from every waypoint on the cached path.
    pub replan_on_path_deviation: bool,
    /// Maximum allowable distance (metres) from any cached waypoint before a
    /// deviation replan is triggered.  Only used when
    /// `replan_on_path_deviation` is `true`.
    pub deviation_tolerance_m: f64,
    /// Pipeline level key written into every produced [`Path`].
    /// Typical values: `"global"`, `"local"`.
    pub level_key: String,
}

// =========================================================================
// == Planner struct ==
// =========================================================================

/// Stateful A\* planner.
///
/// Constructed via [`AStarPlanner::new`] and driven by the autonomy pipeline
/// through the [`Planner`] trait.  The planner owns its search buffers so
/// they are reused across calls without reallocation.
pub struct AStarPlanner {
    config: AStarConfig,
    /// Last-seen goal, used to detect changes and reset replan state.
    last_goal: Option<PlannerGoal>,
    /// Most recently computed path; returned by `current_path()`.
    cached_path: Option<Path>,
    /// Lifecycle status exposed through `Planner::status()`.
    status: PlannerStatus,
    /// Simulation time (seconds) of the last successful plan.
    /// Initialised to `NEG_INFINITY` so the first call always replans.
    last_plan_time: f64,
    /// Pre-allocated A\* search state reused on every `plan()` call.
    buffers: AStarSearchBuffers,
}

impl AStarPlanner {
    /// Construct a new planner with the given configuration.
    ///
    /// Initial status is [`PlannerStatus::Idle`].  A goal must be set via
    /// [`set_goal`](Planner::set_goal) before `plan()` will produce a path.
    pub fn new(config: AStarConfig) -> Self {
        Self {
            config,
            last_goal: None,
            cached_path: None,
            status: PlannerStatus::Idle,
            last_plan_time: f64::NEG_INFINITY,
            buffers: AStarSearchBuffers::new(),
        }
    }
}

// =========================================================================
// == Planner trait impl ==
// =========================================================================

impl SearchPlanner for AStarPlanner {
    /// Compute or update the planned path.
    ///
    /// ## Decision tree
    ///
    /// 1. No goal in inputs → [`PlannerResult::NoGoal`].
    /// 2. Goal changed since last tick → reset status to `Idle`, clear rate gate.
    /// 3. State has no world position → [`PlannerResult::Error`].
    /// 4. Within `arrival_tolerance_m` → [`PlannerResult::GoalReached`].
    /// 5. Rate / deviation gate closed → [`PlannerResult::PathStillValid`].
    /// 6. Map is not `OccupancyGrid2D` → [`PlannerResult::Error`].
    /// 7. Map has zero dimensions → [`PlannerResult::Error`].
    /// 8. Goal outside map → project to boundary; result is [`PlannerResult::GoalOutsideMap`].
    /// 9. Robot outside map → [`PlannerResult::Error`].
    /// 10. A\* finds no path → [`PlannerResult::Unreachable`].
    /// 11. Path found → [`PlannerResult::Path`] (or `GoalOutsideMap` from step 8).
    fn plan(&mut self, now: f64, inputs: &SearchPlannerInputs) -> PlannerResult {
        let goal = match &inputs.goal {
            Some(g) => g.clone(),
            None => return PlannerResult::NoGoal,
        };

        // Reset replan state when the goal changes.
        if self.last_goal.as_ref() != Some(&goal) {
            self.last_goal = Some(goal.clone());
            self.status = PlannerStatus::Idle;
            self.last_plan_time = f64::NEG_INFINITY;
        }

        // Robot 2D position from world-frame state.
        let robot_pos = match inputs.state.get_vector3(&StateVariable::Px(FrameId::World)) {
            Some(p) => Vector2::new(p.x, p.y),
            None => {
                return PlannerResult::Error("AStarPlanner: state missing world position".into())
            }
        };

        let goal_pos = goal.position_2d();

        // Arrival check.
        if (robot_pos - goal_pos).norm() <= self.config.arrival_tolerance_m {
            self.status = PlannerStatus::GoalReached;
            return PlannerResult::GoalReached;
        }

        // Rate gate / deviation check.
        if !self.should_replan(now, inputs) {
            return PlannerResult::PathStillValid;
        }

        // Require an occupancy grid map.
        let (origin, resolution, data) = match &inputs.map {
            MapData::OccupancyGrid2D {
                origin,
                resolution,
                data,
                ..
            } => (origin, resolution, data),
            _ => return PlannerResult::Error("AStarPlanner requires OccupancyGrid2D map".into()),
        };

        let nrows = data.nrows();
        let ncols = data.ncols();
        if nrows == 0 || ncols == 0 {
            return PlannerResult::Error("AStarPlanner: map has zero dimensions".into());
        }

        let space = OccupancyGridSpace {
            origin_x: origin.translation.x,
            origin_y: origin.translation.y,
            resolution: *resolution,
            nrows,
            ncols,
            data,
            threshold: self.config.occupancy_threshold,
        };

        // Project goal to map bounds if it lies outside.
        let mut goal_outside_map = false;
        let effective_goal = if space.is_in_bounds(goal_pos) {
            goal_pos
        } else {
            goal_outside_map = true;
            match space.project_to_bounds(goal_pos) {
                Some(p) => p,
                None => {
                    return PlannerResult::Error(
                        "AStarPlanner: cannot project goal to degenerate map".into(),
                    )
                }
            }
        };

        // Convert world positions to grid cells.
        let (start_row, start_col) = match space.world_to_cell(robot_pos) {
            Some(c) => c,
            None => {
                return PlannerResult::Error(
                    "AStarPlanner: robot position outside map bounds".into(),
                )
            }
        };

        let (goal_row, goal_col) = match space.world_to_cell(effective_goal) {
            Some(c) => c,
            None => {
                return PlannerResult::Error(
                    "AStarPlanner: projected goal outside map bounds".into(),
                )
            }
        };

        // Run A* with pre-allocated buffers.
        let cell_path = match run_astar(
            &space,
            start_row,
            start_col,
            goal_row,
            goal_col,
            self.config.max_search_depth,
            &mut self.buffers,
        ) {
            Some(p) => p,
            None => {
                self.status = PlannerStatus::Failed;
                return PlannerResult::Unreachable;
            }
        };

        // Convert cells to world-frame waypoints.
        let mut waypoints: Vec<_> = cell_path
            .iter()
            .map(|&(row, col)| {
                let wp = space.cell_to_world_center(row, col);
                make_waypoint(wp.x, wp.y, now)
            })
            .collect();

        // Optional string-pull smoothing.
        if self.config.enable_path_smoothing && waypoints.len() > 2 {
            waypoints = smooth_path(&waypoints, &space);
        }

        let path = Path {
            waypoints,
            timestamp: now,
            level_key: self.config.level_key.clone(),
        };

        self.last_plan_time = now;
        self.status = PlannerStatus::Active;
        self.cached_path = Some(path.clone());

        if goal_outside_map {
            PlannerResult::GoalOutsideMap(path)
        } else {
            PlannerResult::Path(path)
        }
    }

    /// Return `true` if a new plan should be computed this tick.
    ///
    /// Replan unconditionally when:
    /// - Status is `Idle` (goal just set, or no path computed yet).
    /// - `cached_path` is `None`.
    ///
    /// When `replan_on_path_deviation` is enabled, also replan if the
    /// minimum distance from the robot to any cached waypoint exceeds
    /// `deviation_tolerance_m`.
    ///
    /// Otherwise replan when `ctx.now - last_plan_time >= 1 / rate_hz`.
    fn should_replan(&self, now: f64, inputs: &SearchPlannerInputs) -> bool {
        if self.status == PlannerStatus::Idle || self.cached_path.is_none() {
            return true;
        }

        let elapsed = now - self.last_plan_time;
        let period = if self.config.rate_hz > 0.0 {
            1.0 / self.config.rate_hz
        } else {
            f64::INFINITY
        };

        // Deviation check (optional).
        if self.config.replan_on_path_deviation {
            if let Some(robot_pos) = inputs.state.get_vector3(&StateVariable::Px(FrameId::World)) {
                if let Some(path) = &self.cached_path {
                    let robot_2d = Vector2::new(robot_pos.x, robot_pos.y);
                    let min_dist = path
                        .waypoints
                        .iter()
                        .map(|wp| {
                            let wp2d = Vector2::new(wp.state.vector[0], wp.state.vector[1]);
                            (robot_2d - wp2d).norm()
                        })
                        .fold(f64::INFINITY, f64::min);
                    if min_dist > self.config.deviation_tolerance_m {
                        return true;
                    }
                }
            }
        }

        elapsed >= period
    }
}

impl AStarPlanner {
    /// Lifecycle status — exposed as an inherent accessor for diagnostics and
    /// tests. Not part of [`SearchPlanner`]; the pipeline reads planner outputs
    /// from the bus, not from the planner directly.
    pub fn status(&self) -> PlannerStatus {
        self.status.clone()
    }

    /// Most recently computed path, if any. Inherent accessor (see
    /// [`Self::status`]).
    pub fn current_path(&self) -> Option<&Path> {
        self.cached_path.as_ref()
    }
}

// =========================================================================
// == Tests ==
// =========================================================================

#[cfg(test)]
mod tests {
    use nalgebra::{DMatrix, Isometry3, Vector2};
    use std::collections::HashMap;

    use crate::frames::{FrameId, RobotState, StateVariable};
    use crate::mapping::MapData;
    use crate::planning::types::{PlannerGoal, PlannerResult, PlannerStatus};
    use crate::planning::SearchPlanner;
    use crate::planning::SearchPlannerInputs;

    use super::{AStarConfig, AStarPlanner};

    // -------------------------------------------------------------------------
    // Test helpers
    // -------------------------------------------------------------------------

    /// Default config: 1 Hz, 0.5 m arrival, threshold=128, 100k depth limit,
    /// smoothing off, deviation off.
    fn default_config() -> AStarConfig {
        AStarConfig {
            rate_hz: 1.0,
            arrival_tolerance_m: 0.5,
            occupancy_threshold: 128,
            max_search_depth: 100_000,
            enable_path_smoothing: false,
            replan_on_path_deviation: false,
            deviation_tolerance_m: 1.0,
            level_key: "global".into(),
        }
    }

    /// Build a minimal world-frame state with only `[Px, Py, Pz]`.
    fn make_state(x: f64, y: f64) -> RobotState {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Pz(FrameId::World),
        ];
        let mut state = RobotState::new(layout, 0.0);
        state.vector[0] = x;
        state.vector[1] = y;
        state.vector[2] = 0.0;
        state
    }

    /// All-zero `nrows × ncols` grid anchored at the world origin.
    fn clear_map(nrows: usize, ncols: usize, resolution: f64) -> MapData {
        MapData::OccupancyGrid2D {
            origin: Isometry3::identity(),
            resolution,
            data: DMatrix::from_element(nrows, ncols, 0u8),
            version: 0,
        }
    }

    fn goal_2d(x: f64, y: f64) -> PlannerGoal {
        PlannerGoal::WorldPosition2D(Vector2::new(x, y))
    }

    fn make_inputs(x: f64, y: f64, map: MapData, goal: Option<PlannerGoal>) -> SearchPlannerInputs {
        SearchPlannerInputs {
            state: make_state(x, y),
            map,
            goal,
        }
    }

    // -------------------------------------------------------------------------
    // plan() result variants
    // -------------------------------------------------------------------------

    /// No goal set → `NoGoal` immediately.
    #[test]
    fn plan_no_goal() {
        let mut planner = AStarPlanner::new(default_config());
        let inputs = make_inputs(0.0, 0.0, clear_map(10, 10, 1.0), None);
        let result = planner.plan(0.0, &inputs);
        assert!(matches!(result, PlannerResult::NoGoal));
    }

    /// Robot at (5.0, 5.0), goal at (5.1, 5.0): distance ≈ 0.1 m < 0.5 m
    /// arrival tolerance → `GoalReached` without running A\*.
    #[test]
    fn plan_goal_reached() {
        let mut planner = AStarPlanner::new(default_config());
        let inputs = make_inputs(5.0, 5.0, clear_map(20, 20, 1.0), Some(goal_2d(5.1, 5.0)));
        let result = planner.plan(0.0, &inputs);
        assert!(matches!(result, PlannerResult::GoalReached));
        assert_eq!(planner.status(), PlannerStatus::GoalReached);
    }

    /// Non-OccupancyGrid2D map variant → `Error`.
    #[test]
    fn plan_wrong_map_type() {
        let mut planner = AStarPlanner::new(default_config());
        let inputs = make_inputs(
            0.0,
            0.0,
            MapData::FeatureMap {
                landmarks: HashMap::new(),
            },
            Some(goal_2d(5.0, 5.0)),
        );
        let result = planner.plan(0.0, &inputs);
        assert!(matches!(result, PlannerResult::Error(_)));
    }

    /// A 0×0 grid → `Error` (zero dimensions guard).
    #[test]
    fn plan_empty_map() {
        let mut planner = AStarPlanner::new(default_config());
        let map = MapData::OccupancyGrid2D {
            origin: Isometry3::identity(),
            resolution: 1.0,
            data: DMatrix::from_element(0, 0, 0u8),
            version: 0,
        };
        let inputs = make_inputs(0.0, 0.0, map, Some(goal_2d(1.0, 1.0)));
        let result = planner.plan(0.0, &inputs);
        assert!(matches!(result, PlannerResult::Error(_)));
    }

    /// Robot at cell (0,0), goal at cell (9,9) on a clear 10×10 grid.
    /// Expect a non-empty `Path` whose last waypoint is near (9.5, 9.5).
    #[test]
    fn plan_success() {
        let mut planner = AStarPlanner::new(default_config());
        let inputs = make_inputs(0.5, 0.5, clear_map(10, 10, 1.0), Some(goal_2d(9.5, 9.5)));
        let result = planner.plan(0.0, &inputs);
        match result {
            PlannerResult::Path(path) => {
                assert!(!path.waypoints.is_empty());
                let last = path.waypoints.last().unwrap();
                // Last waypoint should be at or near the goal cell centre.
                assert!((last.state.vector[0] - 9.5).abs() < 1.0);
                assert!((last.state.vector[1] - 9.5).abs() < 1.0);
            }
            other => panic!("expected Path, got {:?}", std::mem::discriminant(&other)),
        }
        assert_eq!(planner.status(), PlannerStatus::Active);
    }

    /// Cell (0,0) surrounded by obstacles on its three reachable neighbours →
    /// A\* exhausts the open list → `Unreachable`.
    #[test]
    fn plan_unreachable() {
        let mut planner = AStarPlanner::new(default_config());
        let mut data = DMatrix::from_element(10, 10, 0u8);
        data[(0, 1)] = 255;
        data[(1, 0)] = 255;
        data[(1, 1)] = 255;
        let map = MapData::OccupancyGrid2D {
            origin: Isometry3::identity(),
            resolution: 1.0,
            data,
            version: 0,
        };
        let inputs = make_inputs(0.5, 0.5, map, Some(goal_2d(9.5, 9.5)));
        let result = planner.plan(0.0, &inputs);
        assert!(matches!(result, PlannerResult::Unreachable));
        assert_eq!(planner.status(), PlannerStatus::Failed);
    }

    /// Goal at (50, 50) is outside a 10×10 m grid.  The planner projects the
    /// goal to the map boundary and returns `GoalOutsideMap(path)`.
    #[test]
    fn plan_goal_outside_map() {
        let mut planner = AStarPlanner::new(default_config());
        let inputs = make_inputs(0.5, 0.5, clear_map(10, 10, 1.0), Some(goal_2d(50.0, 50.0)));
        let result = planner.plan(0.0, &inputs);
        assert!(matches!(result, PlannerResult::GoalOutsideMap(_)));
    }

    // -------------------------------------------------------------------------
    // should_replan() logic
    // -------------------------------------------------------------------------

    /// Fresh planner has `Idle` status → always replans, regardless of time.
    #[test]
    fn should_replan_idle() {
        let planner = AStarPlanner::new(default_config());
        let inputs = make_inputs(0.0, 0.0, clear_map(10, 10, 1.0), None);
        assert!(planner.should_replan(0.0, &inputs));
    }

    /// After a successful plan at `t=0` with `rate_hz=1.0`, the gate is
    /// closed at `t=0.5` s and opens again at `t=1.0` s.
    #[test]
    fn should_replan_rate_gate() {
        let mut planner = AStarPlanner::new(default_config()); // rate_hz = 1.0
        let map = clear_map(10, 10, 1.0);

        // Seed last_plan_time = 0.0.
        planner.plan(
            0.0,
            &make_inputs(0.5, 0.5, clear_map(10, 10, 1.0), Some(goal_2d(9.5, 9.5))),
        );

        assert!(!planner.should_replan(
            0.5,
            &make_inputs(0.5, 0.5, clear_map(10, 10, 1.0), Some(goal_2d(9.5, 9.5)))
        )); // half-period: gate closed
        assert!(planner.should_replan(1.0, &make_inputs(0.5, 0.5, map, Some(goal_2d(9.5, 9.5)))));
        // full period: gate opens
    }

    /// With `rate_hz=0.0` (gate disabled) and `replan_on_path_deviation=true`,
    /// the planner replans only when the robot strays beyond `deviation_tolerance_m`.
    #[test]
    fn should_replan_deviation() {
        let mut config = default_config();
        config.replan_on_path_deviation = true;
        config.deviation_tolerance_m = 1.0;
        config.rate_hz = 0.0; // Disable rate gate; deviation is the only trigger.

        let mut planner = AStarPlanner::new(config);

        // Compute path at t=0.
        planner.plan(
            0.0,
            &make_inputs(0.5, 0.5, clear_map(10, 10, 1.0), Some(goal_2d(9.5, 9.5))),
        );

        // Robot on the path — no replan.
        assert!(!planner.should_replan(
            0.1,
            &make_inputs(0.5, 0.5, clear_map(10, 10, 1.0), Some(goal_2d(9.5, 9.5)))
        ));
        // Robot far from every waypoint — triggers replan.
        assert!(planner.should_replan(
            0.1,
            &make_inputs(50.0, 50.0, clear_map(10, 10, 1.0), Some(goal_2d(9.5, 9.5)))
        ));
    }
}
