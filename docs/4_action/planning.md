# Path Planning System

## What Was Implemented

### Core Planning Layer (`helios_core`)

A full `Planner` trait replacing the original stub. Planners own their replan decision via `should_replan()` and return a `Path` (full waypoint list) rather than a single `TrajectoryPoint`.

**New types:**
- `PlannerGoal` — `WorldPosition2D`, `WorldPose`, or `GlobalPathWaypoint`
- `PlannerResult` — 7 variants: `Path`, `GoalOutsideMap`, `Unreachable`, `GoalReached`, `PathStillValid`, `NoGoal`, `Error`
- `PlannerStatus` — `Idle`, `Active`, `GoalReached`, `Failed`
- `PlannerContext` — clock + optional TF provider, passed to every `plan()` call
- `Path` — `Vec<TrajectoryPoint>` with timestamp and level key

**A\* on `OccupancyGrid2D`:**
- Rate-gated replanning via `rate_hz`
- Octile heuristic, 8-directional grid search
- Configurable occupancy threshold and max search depth
- Optional Bresenham line-of-sight string-pull smoothing
- `GoalOutsideMap`: projects the goal to the nearest free boundary cell so the robot always has a direction even when the goal is off the current map window

**RRT\* stub (`rrt.rs`):**
Implements the full `Planner` trait but returns `Error("not yet implemented")`. Registered as `"RrtStar"` in the planner registry. This validates that a continuous-space planner fits the same trait and TOML pattern without touching any pipeline or ECS code.

---

### Pipeline Layer (`helios_runtime`)

`ControlCore` gained:
- `cached_paths: HashMap<PipelineLevel, Path>` — most recent plan per level
- `lookahead_indices: HashMap<PipelineLevel, usize>` — current waypoint index per level
- `step_planners()` — runs each leveled planner, updates cached paths
- `advance_lookahead()` — steps the index when the robot is within 2 m of the current waypoint
- `get_cached_path()` / `get_active_lookahead_waypoint()` — read-only accessors for the sim layer
- `set_goal()` — stores a `PlannerGoal` and flags all planners for immediate replan

`step_controllers()` now passes the active look-ahead waypoint as `ControlContext::reference`. All existing controllers (PID, LQR, etc.) work without modification.

`PipelineBuilder::with_goal()` lets the spawning layer inject a static goal at startup.

---

### Simulation Layer (`helios_sim`)

**Planner registry:**
`PlannerFactory`, `PlannerBuildContext`, and `DefaultPlannersPlugin` follow the same pattern as the controller registry. Adding a planner is one `register_planner` call in `DefaultPlannersPlugin` — zero spawning systems change.

**`PlanningPlugin`:**
- `planning_system` (FixedUpdate / `SimulationSet::Planning`) — builds a `HashMap<PipelineLevel, &MapData>` from `EstimatorComponent` (SLAM map) and `MapperComponent`, then calls `step_planners()`
- `goal_command_system` (FixedUpdate / `SimulationSet::Behavior`) — reads `GoalCommandEvent`, calls `set_goal()`, and keeps `GoalRegistry` in sync

**Click interaction (`interaction.rs`):**
- Left-click near an agent → selects it (`SelectedAgent` resource)
- Right-click with an agent selected → ray-ground-plane intersection → ENU conversion → `GoalCommandEvent` for that specific agent
- Escape → deselects
- `GoalRegistry` tracks the most recent goal per agent entity

**Gizmos:**

| Visual | Meaning |
|--------|---------|
| Yellow polyline | Local planned path |
| Orange polyline | Global planned path |
| Cyan sphere | Active look-ahead waypoint |
| Green ring | Selected agent |
| Magenta sphere + line | Commanded goal |

Toggle with **F9** (planned path) — selection/goal markers are always visible when an agent is selected.

---

## Known Gaps and Improvements

### Mapper is `NoneMapper` by default
The A\* planner receives `MapData::None` and returns `PlannerResult::Error`. The robot moves toward `goal_pose` only because `step_controllers()` runs with `reference: None` — the same behavior as before planning was added. **Until an `OccupancyGridMapper` is wired in, paths are never computed from a map.** This is the single biggest gap between "planner exists" and "planner works."

### Look-ahead radius is a fixed constant
The 2 m advance threshold works for slow open-space scenarios but is too tight for high-speed driving or narrow corridors. It should be a per-planner config field, or ideally computed as `velocity × lookahead_time_s` to scale naturally with speed.

### `GoalOutsideMap` has no continuation
The planner projects the goal to the map boundary and returns a path to that edge. Once the robot reaches the edge, nothing re-triggers a replan with the real goal. Needs either an automatic replan when the map window shifts to include the original goal, or a `reached_map_edge` status that the pipeline handles explicitly.

### Path deviation detection is naive
The current deviation check measures distance from the robot to the nearest *waypoint*. It should measure perpendicular distance to the nearest *path segment*, which is more accurate and stable — especially on curves where waypoints are sparse.

### No planner telemetry
`PlannerStatus`, replanning events, and path length are not published to the `TopicBus`. Tuning `rate_hz` and `deviation_tolerance_m` is currently blind — no Foxglove visibility into whether the planner is replanning, failing, or reaching goals.

### Selection state not reflected in the debug legend
The F9 entry in the legend tracks the path gizmo toggle but does not show which agent is currently selected or whether a goal is pending.

---

## Future Direction

### Near-term: make the planner actually function

- **Wire an `OccupancyGridMapper`** — `LidarPlugin` already raycasts; the mapper needs to accumulate hits into `DMatrix<u8>` and expose it via `get_map(PipelineLevel::Local)`.
- **Static map loader** — read a PNG/PGM occupancy grid from TOML config so A\* can be tested on a known map without live sensor data.

### Mid-term: robust two-level planning

- **Global + local planner pair** — Global A\* on a static map at 1 Hz gives the coarse route; local A\* or DWA on the live rolling-window map at 5–10 Hz handles dynamic obstacle avoidance. `ControlCore` already holds `cached_paths` for both `PipelineLevel::Global` and `PipelineLevel::Local`.
- **`GlobalPathWaypoint` goal chaining** — the local planner receives `PlannerGoal::GlobalPathWaypoint { index }` and automatically advances through the global route on `GoalReached`, keeping the global planner decoupled from the local one.
- **Velocity-aware waypoints** — `TrajectoryPoint` has slots for velocity; A\* currently sets them to zero. A post-processing pass could assign target speeds based on path curvature (slow on turns, fast on straights).

### Longer-term

- **RRT\* / RRT-Connect** — continuous-space planning for large outdoor environments or 3D paths where a grid is too coarse. The trait and TOML registry pattern are already in place; only the tree-building logic needs implementing.
- **Map-drift replanning** — as the rolling-window map shifts, invalidate cached path segments that are no longer in-bounds and replan immediately rather than waiting for the rate gate.
- **Multi-goal missions** — a `MissionComponent` in `helios_sim` holds a sequence of `PlannerGoal`s and fires `GoalCommandEvent`s on each `GoalReached`. The planner itself stays single-goal; mission sequencing is a sim-layer concern.
- **Hardware portability** — `ControlCore::step_planners()` and the full `Planner` trait are Bevy-free. Running A\* on `helios_hw` requires only replacing the map source (live sensor → same `MapData` type) and the runtime (`SimRuntime` → hardware clock + TF). No algorithm code changes.
