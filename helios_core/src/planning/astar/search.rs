// helios_core/src/planning/astar/search.rs

//! Core A\* search: open-list node, pre-allocated buffers, octile heuristic,
//! and the search loop.
//!
//! ## Pre-allocation rationale
//!
//! A* requires a priority queue (`open`), a best-known cost map (`g_score`),
//! and a parent pointer map (`parent`). At 1–10 Hz on a 100×100 grid all
//! three collections are non-trivially large. Creating them fresh on every
//! `plan()` call causes repeated heap allocation and zeroing.
//!
//! [`AStarSearchBuffers`] is stored on the planner and reused across calls:
//! each search begins with `buffers.clear()`, which empties the collections
//! without releasing their backing storage.
//!
//! ## Stale-entry trick
//!
//! Rather than removing entries from the binary heap when a shorter path is
//! found, the search pushes a second entry with the lower `g_score`. When the
//! stale entry is eventually popped, the guard
//! `node.g_score > best_g + 1e-9` discards it cheaply.

use std::collections::{BinaryHeap, HashMap};

use super::grid_space::OccupancyGridSpace;

// =========================================================================
// == Pre-allocated search buffers ==
// =========================================================================

/// Heap-allocated search state reused across `plan()` calls to avoid
/// per-call allocation churn on the [`BinaryHeap`] and two [`HashMap`]s.
///
/// Call [`clear`](AStarSearchBuffers::clear) at the top of each search; the
/// backing storage is retained between calls.
pub(super) struct AStarSearchBuffers {
    /// Priority queue of nodes ordered by ascending `f_score`.
    pub open: BinaryHeap<AStarNode>,
    /// Best known path cost from start to each visited cell.
    pub g_score: HashMap<(usize, usize), f64>,
    /// Back-pointer map used to reconstruct the path on goal expansion.
    pub parent: HashMap<(usize, usize), (usize, usize)>,
}

impl AStarSearchBuffers {
    /// Allocate empty collections. Called once when [`AStarPlanner`] is constructed.
    pub fn new() -> Self {
        Self {
            open: BinaryHeap::new(),
            g_score: HashMap::new(),
            parent: HashMap::new(),
        }
    }

    /// Empty all three collections without releasing backing storage.
    pub fn clear(&mut self) {
        self.open.clear();
        self.g_score.clear();
        self.parent.clear();
    }
}

// =========================================================================
// == Open-list node ==
// =========================================================================

/// A single entry in the A\* open list.
///
/// [`BinaryHeap`] is a max-heap; the [`Ord`] impl inverts the comparison on
/// `f_score` so the node with the *lowest* `f` is popped first (min-heap
/// behaviour). Ties are broken by preferring *higher* `g_score`, which biases
/// expansion toward nodes closer to the goal.
#[derive(PartialEq)]
pub(super) struct AStarNode {
    /// `g + h` — estimated total path cost through this node.
    pub f_score: f64,
    /// Cost paid to reach this node from the start.
    pub g_score: f64,
    /// Grid row of this node.
    pub row: usize,
    /// Grid column of this node.
    pub col: usize,
}

impl Eq for AStarNode {}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Invert f_score comparison → min-heap; break ties by preferring higher g.
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

// =========================================================================
// == Heuristic ==
// =========================================================================

/// Octile distance heuristic — admissible and consistent on an 8-connected grid.
///
/// For a grid where cardinal moves cost `1` and diagonal moves cost `√2`, the
/// true shortest distance between two cells is exactly the octile distance:
///
/// ```text
/// h = |dr - dc| + min(dr, dc) * √2
/// ```
///
/// where `dr` and `dc` are the absolute row and column differences.
pub(super) fn octile_heuristic(r: usize, c: usize, goal_r: usize, goal_c: usize) -> f64 {
    let dr = (r as f64 - goal_r as f64).abs();
    let dc = (c as f64 - goal_c as f64).abs();
    (dr - dc).abs() + dc.min(dr) * std::f64::consts::SQRT_2
}

// =========================================================================
// == Core search ==
// =========================================================================

/// Run A\* on `space` from `(start_row, start_col)` to `(goal_row, goal_col)`.
///
/// Returns the cell path — a `Vec` of `(row, col)` pairs from start to goal
/// inclusive — or `None` if the goal is unreachable or `max_depth` node
/// expansions are exhausted.
///
/// `buffers` is cleared at entry and its collections are reused in-place to
/// avoid allocation overhead on repeated calls.
///
/// ## Step costs
/// * Cardinal move (Δrow or Δcol = 1): cost `1.0`.
/// * Diagonal move (Δrow = Δcol = 1): cost `√2`.
pub(super) fn run_astar(
    space: &OccupancyGridSpace,
    start_row: usize,
    start_col: usize,
    goal_row: usize,
    goal_col: usize,
    max_depth: usize,
    buffers: &mut AStarSearchBuffers,
) -> Option<Vec<(usize, usize)>> {
    buffers.clear();

    buffers.g_score.insert((start_row, start_col), 0.0);
    buffers.open.push(AStarNode {
        f_score: octile_heuristic(start_row, start_col, goal_row, goal_col),
        g_score: 0.0,
        row: start_row,
        col: start_col,
    });

    let mut iterations = 0usize;

    while let Some(node) = buffers.open.pop() {
        iterations += 1;
        if iterations > max_depth {
            return None;
        }

        let (row, col) = (node.row, node.col);

        if row == goal_row && col == goal_col {
            // Reconstruct path by walking parent pointers back to start.
            let mut path = vec![(row, col)];
            let mut cur = (row, col);
            while let Some(&par) = buffers.parent.get(&cur) {
                path.push(par);
                cur = par;
            }
            path.reverse();
            return Some(path);
        }

        // Discard stale open-list entries (a shorter path was found later).
        let best_g = *buffers.g_score.get(&(row, col)).unwrap_or(&f64::INFINITY);
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

            let existing_g = *buffers.g_score.get(&(nr, nc)).unwrap_or(&f64::INFINITY);
            if new_g < existing_g - 1e-9 {
                buffers.g_score.insert((nr, nc), new_g);
                buffers.parent.insert((nr, nc), (row, col));
                let f = new_g + octile_heuristic(nr, nc, goal_row, goal_col);
                buffers.open.push(AStarNode { f_score: f, g_score: new_g, row: nr, col: nc });
            }
        }
    }

    None
}

// =========================================================================
// == Tests ==
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::DMatrix;

    /// All-zero grid of the given size (every cell free, threshold=128).
    fn clear_grid(nrows: usize, ncols: usize) -> DMatrix<u8> {
        DMatrix::from_element(nrows, ncols, 0u8)
    }

    /// Unit-resolution space anchored at the world origin.
    fn make_space<'a>(data: &'a DMatrix<u8>, nrows: usize, ncols: usize) -> OccupancyGridSpace<'a> {
        OccupancyGridSpace {
            origin_x: 0.0,
            origin_y: 0.0,
            resolution: 1.0,
            nrows,
            ncols,
            data,
            threshold: 128,
        }
    }

    /// Straight-line octile distance equals the Manhattan distance when
    /// movement is axis-aligned (no diagonal component).
    #[test]
    fn octile_heuristic_straight() {
        assert!((octile_heuristic(0, 0, 5, 0) - 5.0).abs() < 1e-9);
        assert!((octile_heuristic(0, 0, 0, 5) - 5.0).abs() < 1e-9);
    }

    /// A perfectly diagonal path of `n` cells costs `n * √2`.
    #[test]
    fn octile_heuristic_diagonal() {
        let expected = 4.0 * std::f64::consts::SQRT_2;
        assert!((octile_heuristic(0, 0, 4, 4) - expected).abs() < 1e-9);
    }

    /// On a clear 5×5 grid A\* must find a path from `(0,0)` to `(4,4)`.
    /// The returned path must begin at the start cell and end at the goal cell.
    #[test]
    fn run_astar_simple() {
        let data = clear_grid(5, 5);
        let space = make_space(&data, 5, 5);
        let mut bufs = AStarSearchBuffers::new();
        let path = run_astar(&space, 0, 0, 4, 4, 10000, &mut bufs);
        let p = path.expect("path should be found on clear 5×5 grid");
        assert_eq!(p[0], (0, 0));
        assert_eq!(*p.last().unwrap(), (4, 4));
    }

    /// A wall at `col=2`, rows 0–3, with a gap at `row=4` must force the
    /// path through the gap cell `(4, 2)`.
    #[test]
    fn run_astar_with_obstacle() {
        let mut data = DMatrix::from_element(5, 5, 0u8);
        for r in 0..4 {
            data[(r, 2)] = 255;
        }
        let space = make_space(&data, 5, 5);
        let mut bufs = AStarSearchBuffers::new();
        let path = run_astar(&space, 0, 0, 0, 4, 10000, &mut bufs);
        let p = path.expect("path should go through gap at row=4");
        assert!(p.iter().any(|&(r, c)| r == 4 && c == 2));
    }

    /// Start cell `(0,0)` walled off on all three adjacent cells — no path exists.
    #[test]
    fn run_astar_unreachable() {
        let mut data = DMatrix::from_element(5, 5, 0u8);
        data[(0, 1)] = 255;
        data[(1, 0)] = 255;
        data[(1, 1)] = 255;
        let space = make_space(&data, 5, 5);
        let mut bufs = AStarSearchBuffers::new();
        let path = run_astar(&space, 0, 0, 4, 4, 10000, &mut bufs);
        assert!(path.is_none());
    }

    /// With `max_depth=1` the search may pop the start node but will never
    /// reach the distant goal before the iteration cap fires.
    #[test]
    fn run_astar_max_depth() {
        let data = clear_grid(20, 20);
        let space = make_space(&data, 20, 20);
        let mut bufs = AStarSearchBuffers::new();
        let path = run_astar(&space, 0, 0, 19, 19, 1, &mut bufs);
        assert!(path.is_none());
    }

    /// When start equals goal the search exits immediately on the first pop
    /// and returns a single-element path.
    #[test]
    fn run_astar_start_equals_goal() {
        let data = clear_grid(5, 5);
        let space = make_space(&data, 5, 5);
        let mut bufs = AStarSearchBuffers::new();
        let path = run_astar(&space, 2, 2, 2, 2, 10000, &mut bufs);
        assert_eq!(path, Some(vec![(2, 2)]));
    }
}
