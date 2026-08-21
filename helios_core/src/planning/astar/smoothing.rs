//! String-pull path smoothing and waypoint construction.
//!
//! ## String-pull algorithm
//!
//! Raw A\* output is a staircase of axis-aligned and diagonal cells.  The
//! string-pull (or "funnel") heuristic removes redundant intermediate
//! waypoints:
//!
//! 1. Start at `anchor = waypoints[0]`.
//! 2. Walk forward through the remaining waypoints.  For each `probe`, check
//!    whether there is a clear Bresenham line-of-sight between `anchor` and
//!    `probe`.
//! 3. Keep the furthest `probe` that has clear LOS from `anchor` as the next
//!    waypoint; advance `anchor` to it.
//! 4. Repeat until the last waypoint is reached.
//!
//! This typically halves the waypoint count on open terrain while preserving
//! all obstacle-required turns.
//!
//! ## Waypoint format
//!
//! Each waypoint is a [`Point<Enu>`]: a pure ENU world-frame position. A path
//! is geometry only — no time, velocity, or heading — so the followers and
//! visualisers read the position directly off the typed point.

use super::grid_space::OccupancyGridSpace;

use crate::frames::conventions::Enu;
use crate::frames::quantities::Point;

use nalgebra::Vector2;

// =========================================================================
// == String-pull smoothing ==
// =========================================================================

/// Remove redundant waypoints using Bresenham line-of-sight tests.
///
/// Returns a new `Vec<Point<Enu>>` containing only the waypoints that
/// cannot be skipped without crossing an obstacle.  If there are ≤ 2
/// waypoints the input is returned unchanged.
///
/// The cells corresponding to each waypoint are computed once via
/// [`OccupancyGridSpace::world_to_cell`]; waypoints that fall outside the
/// map are silently dropped from the cell list (they will not be used as
/// LOS anchors or probes).
pub(super) fn smooth_path(waypoints: &[Point<Enu>], space: &OccupancyGridSpace) -> Vec<Point<Enu>> {
    if waypoints.len() <= 2 {
        return waypoints.to_vec();
    }

    let cells: Vec<(usize, usize)> = waypoints
        .iter()
        .filter_map(|wp| {
            let pos = Vector2::new(wp.x(), wp.y());
            space.world_to_cell(pos)
        })
        .collect();

    let mut smoothed = vec![waypoints[0].clone()];
    let mut anchor = 0usize;

    while anchor < cells.len() - 1 {
        // Scan forward to find the last probe reachable from `anchor`.
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

/// Build a waypoint at `(world_x, world_y, world_z)` in the ENU world frame.
///
/// A waypoint is a [`Point<Enu>`] — position only. The planner emits no
/// velocity, heading, or timing; a path is geometry, and any dynamics belong
/// to a trajectory produced by a later optimizer, not to this string-pull.
pub(super) fn make_waypoint(world_x: f64, world_y: f64, world_z: f64) -> Point<Enu> {
    Point::new(world_x, world_y, world_z)
}

// =========================================================================
// == Tests ==
// =========================================================================

#[cfg(test)]
mod tests {
    use super::super::grid_space::OccupancyGridSpace;
    use super::*;
    use nalgebra::DMatrix;

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

    /// Shorthand: build a waypoint at `(x, y)` on the ground plane (`z = 0`).
    fn wp(x: f64, y: f64) -> Point<Enu> {
        make_waypoint(x, y, 0.0)
    }

    /// A 2-waypoint path is already minimal; the smoother must return it unchanged.
    #[test]
    fn smooth_path_trivial() {
        let data = DMatrix::from_element(10, 10, 0u8);
        let space = make_space(&data, 10, 10);
        let wps = vec![wp(0.5, 0.5), wp(9.5, 9.5)];
        let result = smooth_path(&wps, &space);
        assert_eq!(result.len(), 2);
    }

    /// Three collinear waypoints on the same grid row with a clear map: the
    /// LOS from `(0,0)` reaches `(0,9)` directly, so the middle waypoint
    /// `(0,4)` is pruned and only 2 waypoints remain.
    #[test]
    fn smooth_path_collinear() {
        let data = DMatrix::from_element(10, 10, 0u8);
        let space = make_space(&data, 10, 10);
        // Cells: wp(0.5,0.5)→(row=0,col=0), wp(4.5,0.5)→(0,4), wp(9.5,0.5)→(0,9).
        let wps = vec![wp(0.5, 0.5), wp(4.5, 0.5), wp(9.5, 0.5)];
        let result = smooth_path(&wps, &space);
        assert_eq!(result.len(), 2);
    }

    /// A wall at `col=5`, rows 0–4 blocks the direct LOS from `(0,0)` to
    /// `(0,9)` and also from `(0,4)` to `(0,9)`.  The middle waypoint at
    /// `(0,4)` is load-bearing and must not be pruned; all 3 waypoints remain.
    #[test]
    fn smooth_path_obstacle() {
        let mut data = DMatrix::from_element(10, 10, 0u8);
        for r in 0..5 {
            data[(r, 5)] = 255;
        }
        let space = make_space(&data, 10, 10);
        // Cells: (0,0)→(0,4)→(0,9); wall at col=5 rows 0-4 blocks both LOS jumps.
        let wps = vec![wp(0.5, 0.5), wp(4.5, 0.5), wp(9.5, 0.5)];
        let result = smooth_path(&wps, &space);
        assert_eq!(result.len(), 3);
    }
}
