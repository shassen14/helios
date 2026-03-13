// helios_core/src/planning/astar.rs
//
// A* path planner on OccupancyGrid2D maps.

use std::collections::{BinaryHeap, HashMap};

use nalgebra::{DMatrix, Vector2};

use crate::control::TrajectoryPoint;
use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::mapping::MapData;

use super::context::PlannerContext;
use super::search_space::SearchSpace;
use super::types::{Path, PlannerGoal, PlannerResult, PlannerStatus};
use super::Planner;

// =========================================================================
// == Config ==
// =========================================================================

pub struct AStarConfig {
    pub rate_hz: f64,
    pub arrival_tolerance_m: f64,
    pub occupancy_threshold: u8,
    pub max_search_depth: usize,
    pub enable_path_smoothing: bool,
    pub replan_on_path_deviation: bool,
    pub deviation_tolerance_m: f64,
    pub level_key: String,
}

// =========================================================================
// == Planner struct ==
// =========================================================================

pub struct AStarPlanner {
    config: AStarConfig,
    goal: Option<PlannerGoal>,
    cached_path: Option<Path>,
    status: PlannerStatus,
    last_plan_time: f64,
}

impl AStarPlanner {
    pub fn new(config: AStarConfig) -> Self {
        Self {
            config,
            goal: None,
            cached_path: None,
            status: PlannerStatus::Idle,
            last_plan_time: f64::NEG_INFINITY,
        }
    }
}

// =========================================================================
// == Internal grid search space ==
// =========================================================================

struct OccupancyGridSpace<'a> {
    origin_x: f64,
    origin_y: f64,
    resolution: f64,
    nrows: usize,
    ncols: usize,
    data: &'a DMatrix<u8>,
    threshold: u8,
}

impl<'a> OccupancyGridSpace<'a> {
    fn world_to_cell(&self, pos: Vector2<f64>) -> Option<(usize, usize)> {
        let col = ((pos.x - self.origin_x) / self.resolution).floor() as isize;
        let row = ((pos.y - self.origin_y) / self.resolution).floor() as isize;
        if col >= 0
            && row >= 0
            && (col as usize) < self.ncols
            && (row as usize) < self.nrows
        {
            Some((row as usize, col as usize))
        } else {
            None
        }
    }

    fn cell_to_world_center(&self, row: usize, col: usize) -> Vector2<f64> {
        Vector2::new(
            self.origin_x + (col as f64 + 0.5) * self.resolution,
            self.origin_y + (row as f64 + 0.5) * self.resolution,
        )
    }

    fn is_cell_free(&self, row: usize, col: usize) -> bool {
        self.data[(row, col)] < self.threshold
    }

    fn neighbors(&self, row: usize, col: usize) -> impl Iterator<Item = (usize, usize)> {
        const DIRS: [(i32, i32); 8] = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ];
        let nrows = self.nrows;
        let ncols = self.ncols;
        DIRS.iter().filter_map(move |&(dr, dc)| {
            let nr = row as i32 + dr;
            let nc = col as i32 + dc;
            if nr >= 0 && nc >= 0 && (nr as usize) < nrows && (nc as usize) < ncols {
                Some((nr as usize, nc as usize))
            } else {
                None
            }
        })
    }

    /// Bresenham line-of-sight check between two cells. Returns true if no obstacle.
    fn has_line_of_sight(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> bool {
        let (mut r, mut c) = (r1 as i32, c1 as i32);
        let (er, ec) = (r2 as i32, c2 as i32);
        let dr = (er - r).abs();
        let dc = (ec - c).abs();
        let sr = if r < er { 1 } else { -1 };
        let sc = if c < ec { 1 } else { -1 };
        let mut err = dr - dc;

        loop {
            if r == er && c == ec {
                break;
            }
            if !self.is_cell_free(r as usize, c as usize) {
                return false;
            }
            let e2 = 2 * err;
            if e2 > -dc {
                err -= dc;
                r += sr;
            }
            if e2 < dr {
                err += dr;
                c += sc;
            }
        }
        true
    }
}

impl<'a> SearchSpace for OccupancyGridSpace<'a> {
    fn is_free(&self, pos: Vector2<f64>) -> bool {
        match self.world_to_cell(pos) {
            Some((row, col)) => self.is_cell_free(row, col),
            None => false,
        }
    }

    fn is_in_bounds(&self, pos: Vector2<f64>) -> bool {
        self.world_to_cell(pos).is_some()
    }

    fn project_to_bounds(&self, pos: Vector2<f64>) -> Option<Vector2<f64>> {
        if self.nrows == 0 || self.ncols == 0 {
            return None;
        }
        let half_cell = self.resolution * 0.5;
        let x_min = self.origin_x + half_cell;
        let x_max = self.origin_x + self.ncols as f64 * self.resolution - half_cell;
        let y_min = self.origin_y + half_cell;
        let y_max = self.origin_y + self.nrows as f64 * self.resolution - half_cell;
        Some(Vector2::new(
            pos.x.clamp(x_min, x_max),
            pos.y.clamp(y_min, y_max),
        ))
    }

    fn resolution(&self) -> Option<f64> {
        Some(self.resolution)
    }
}

// =========================================================================
// == A* search ==
// =========================================================================

/// Node for the A* open list. Min-heap on f_score.
#[derive(PartialEq)]
struct AStarNode {
    f_score: f64,
    g_score: f64,
    row: usize,
    col: usize,
}

impl Eq for AStarNode {}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Max-heap inverted to min-heap via Reverse; fall back on g_score as tiebreaker.
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                other
                    .g_score
                    .partial_cmp(&self.g_score)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }
}

fn octile_heuristic(r: usize, c: usize, goal_r: usize, goal_c: usize) -> f64 {
    let dr = (r as f64 - goal_r as f64).abs();
    let dc = (c as f64 - goal_c as f64).abs();
    (dr - dc).abs() + dc.min(dr) * std::f64::consts::SQRT_2
}

fn run_astar(
    space: &OccupancyGridSpace,
    start_row: usize,
    start_col: usize,
    goal_row: usize,
    goal_col: usize,
    max_depth: usize,
) -> Option<Vec<(usize, usize)>> {
    let mut open: BinaryHeap<AStarNode> = BinaryHeap::new();
    let mut g_score: HashMap<(usize, usize), f64> = HashMap::new();
    let mut parent: HashMap<(usize, usize), (usize, usize)> = HashMap::new();

    g_score.insert((start_row, start_col), 0.0);
    open.push(AStarNode {
        f_score: octile_heuristic(start_row, start_col, goal_row, goal_col),
        g_score: 0.0,
        row: start_row,
        col: start_col,
    });

    let mut iterations = 0usize;

    while let Some(node) = open.pop() {
        iterations += 1;
        if iterations > max_depth {
            return None;
        }

        let (row, col) = (node.row, node.col);

        if row == goal_row && col == goal_col {
            let mut path = vec![(row, col)];
            let mut cur = (row, col);
            while let Some(&par) = parent.get(&cur) {
                path.push(par);
                cur = par;
            }
            path.reverse();
            return Some(path);
        }

        // Skip stale open-list entries.
        let best_g = *g_score.get(&(row, col)).unwrap_or(&f64::INFINITY);
        if node.g_score > best_g + 1e-9 {
            continue;
        }

        for (nr, nc) in space.neighbors(row, col) {
            if !space.is_cell_free(nr, nc) {
                continue;
            }
            let dr = (nr as i32 - row as i32).abs();
            let dc = (nc as i32 - col as i32).abs();
            let step_cost = if dr + dc == 2 { std::f64::consts::SQRT_2 } else { 1.0 };
            let new_g = node.g_score + step_cost;

            let existing_g = *g_score.get(&(nr, nc)).unwrap_or(&f64::INFINITY);
            if new_g < existing_g - 1e-9 {
                g_score.insert((nr, nc), new_g);
                parent.insert((nr, nc), (row, col));
                let f = new_g + octile_heuristic(nr, nc, goal_row, goal_col);
                open.push(AStarNode { f_score: f, g_score: new_g, row: nr, col: nc });
            }
        }
    }

    None
}

// =========================================================================
// == Path smoothing (string-pull) ==
// =========================================================================

fn smooth_path(
    waypoints: &[TrajectoryPoint],
    space: &OccupancyGridSpace,
) -> Vec<TrajectoryPoint> {
    if waypoints.len() <= 2 {
        return waypoints.to_vec();
    }

    let cells: Vec<(usize, usize)> = waypoints
        .iter()
        .filter_map(|wp| {
            let pos = Vector2::new(wp.state.vector[0], wp.state.vector[1]);
            space.world_to_cell(pos)
        })
        .collect();

    let mut smoothed = vec![waypoints[0].clone()];
    let mut anchor = 0usize;

    while anchor < cells.len() - 1 {
        // Find the furthest visible waypoint from anchor.
        let mut furthest = anchor + 1;
        for probe in (anchor + 1)..cells.len() {
            let (ar, ac) = cells[anchor];
            let (pr, pc) = cells[probe];
            if space.has_line_of_sight(ar, ac, pr, pc) {
                furthest = probe;
            }
        }
        smoothed.push(waypoints[furthest].clone());
        anchor = furthest;
    }

    smoothed
}

// =========================================================================
// == Waypoint construction ==
// =========================================================================

fn make_waypoint(world_x: f64, world_y: f64, time: f64) -> TrajectoryPoint {
    let layout = vec![
        StateVariable::Px(FrameId::World),
        StateVariable::Py(FrameId::World),
        StateVariable::Pz(FrameId::World),
    ];
    let mut state = FrameAwareState::new(layout, 0.0, time);
    state.vector[0] = world_x;
    state.vector[1] = world_y;
    state.vector[2] = 0.0;
    TrajectoryPoint { state, state_dot: None, time }
}

// =========================================================================
// == Planner trait impl ==
// =========================================================================

impl Planner for AStarPlanner {
    fn set_goal(&mut self, goal: PlannerGoal) {
        self.goal = Some(goal);
        self.status = PlannerStatus::Idle;
        self.last_plan_time = f64::NEG_INFINITY; // Force immediate replan.
    }

    fn plan(
        &mut self,
        state: &FrameAwareState,
        map: &MapData,
        ctx: &PlannerContext,
    ) -> PlannerResult {
        let goal = match &self.goal {
            Some(g) => g.clone(),
            None => return PlannerResult::NoGoal,
        };

        // Robot 2D position from world-frame state.
        let robot_pos = match state.get_vector3(&StateVariable::Px(FrameId::World)) {
            Some(p) => Vector2::new(p.x, p.y),
            None => {
                return PlannerResult::Error(
                    "AStarPlanner: state missing world position".into(),
                )
            }
        };

        let goal_pos = goal.position_2d();

        // Arrival check.
        if (robot_pos - goal_pos).norm() <= self.config.arrival_tolerance_m {
            self.status = PlannerStatus::GoalReached;
            return PlannerResult::GoalReached;
        }

        // Rate gate / deviation check.
        if !self.should_replan(state, ctx) {
            return PlannerResult::PathStillValid;
        }

        // Require an occupancy grid map.
        let (origin, resolution, data) = match map {
            MapData::OccupancyGrid2D { origin, resolution, data } => (origin, *resolution, data),
            _ => {
                return PlannerResult::Error(
                    "AStarPlanner requires OccupancyGrid2D map".into(),
                )
            }
        };

        let nrows = data.nrows();
        let ncols = data.ncols();
        if nrows == 0 || ncols == 0 {
            return PlannerResult::Error("AStarPlanner: map has zero dimensions".into());
        }

        let space = OccupancyGridSpace {
            origin_x: origin.translation.x,
            origin_y: origin.translation.y,
            resolution,
            nrows,
            ncols,
            data,
            threshold: self.config.occupancy_threshold,
        };

        // Project goal to map bounds if needed.
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

        // Convert positions to grid cells.
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

        // Run A*.
        let cell_path = match run_astar(
            &space,
            start_row,
            start_col,
            goal_row,
            goal_col,
            self.config.max_search_depth,
        ) {
            Some(p) => p,
            None => {
                self.status = PlannerStatus::Failed;
                return PlannerResult::Unreachable;
            }
        };

        // Convert cells to world waypoints.
        let mut waypoints: Vec<TrajectoryPoint> = cell_path
            .iter()
            .map(|&(row, col)| {
                let wp = space.cell_to_world_center(row, col);
                make_waypoint(wp.x, wp.y, ctx.now)
            })
            .collect();

        // Optional string-pull smoothing.
        if self.config.enable_path_smoothing && waypoints.len() > 2 {
            waypoints = smooth_path(&waypoints, &space);
        }

        let path = Path {
            waypoints,
            timestamp: ctx.now,
            level_key: self.config.level_key.clone(),
        };

        self.last_plan_time = ctx.now;
        self.status = PlannerStatus::Active;
        self.cached_path = Some(path.clone());

        if goal_outside_map {
            PlannerResult::GoalOutsideMap(path)
        } else {
            PlannerResult::Path(path)
        }
    }

    fn should_replan(&self, state: &FrameAwareState, ctx: &PlannerContext) -> bool {
        // Always replan if idle (no path yet, or goal just set).
        if self.status == PlannerStatus::Idle || self.cached_path.is_none() {
            return true;
        }

        let elapsed = ctx.now - self.last_plan_time;
        let period = if self.config.rate_hz > 0.0 { 1.0 / self.config.rate_hz } else { f64::INFINITY };

        // Deviation check (optional).
        if self.config.replan_on_path_deviation {
            if let Some(robot_pos) =
                state.get_vector3(&StateVariable::Px(FrameId::World))
            {
                if let Some(path) = &self.cached_path {
                    let robot_2d = Vector2::new(robot_pos.x, robot_pos.y);
                    let min_dist = path
                        .waypoints
                        .iter()
                        .map(|wp| {
                            let wp2d =
                                Vector2::new(wp.state.vector[0], wp.state.vector[1]);
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

    fn status(&self) -> PlannerStatus {
        self.status.clone()
    }

    fn current_path(&self) -> Option<&Path> {
        self.cached_path.as_ref()
    }
}
