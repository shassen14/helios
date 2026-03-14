// helios_core/src/planning/astar/grid_space.rs

//! Grid-space adapter for A\* search.
//!
//! [`OccupancyGridSpace`] wraps a borrowed [`DMatrix<u8>`] and exposes the
//! three operations the search loop needs:
//!
//! 1. **Coordinate conversion** — ENU world ↔ `(row, col)` cell indices.
//! 2. **Free-cell query** — compare raw occupancy value against `threshold`.
//! 3. **8-connected neighbour enumeration** — clipped to grid bounds.
//! 4. **Bresenham line-of-sight** — used by the string-pull smoother.
//!
//! It also implements [`SearchSpace`] so the planner can delegate
//! `is_in_bounds` / `project_to_bounds` / `resolution` queries through the
//! trait interface.
//!
//! ## Coordinate conventions
//! * World frame: ENU (`+X` East, `+Y` North).
//! * Cell indices: `(row, col)` where `row` grows with `+Y` and `col` grows
//!   with `+X`, matching nalgebra's `DMatrix` layout.
//! * Cell origin is the south-west corner; `cell_to_world_center` returns the
//!   centre of the cell (offset by `0.5 * resolution` in each axis).

use nalgebra::{DMatrix, Vector2};

use crate::planning::search_space::SearchSpace;

// =========================================================================
// == OccupancyGridSpace ==
// =========================================================================

/// A view into an [`OccupancyGrid2D`](crate::mapping::MapData) map that
/// provides grid-space operations for the A\* search loop.
///
/// The struct borrows the raw `DMatrix<u8>` from the map for the duration of
/// one `plan()` call; it owns no heap allocations.
pub(super) struct OccupancyGridSpace<'a> {
    /// World-frame X coordinate of the grid's south-west corner (metres).
    pub origin_x: f64,
    /// World-frame Y coordinate of the grid's south-west corner (metres).
    pub origin_y: f64,
    /// Side length of one square cell (metres).
    pub resolution: f64,
    /// Number of rows in the grid (Y extent / resolution).
    pub nrows: usize,
    /// Number of columns in the grid (X extent / resolution).
    pub ncols: usize,
    /// Raw occupancy data. Index as `data[(row, col)]`.
    pub data: &'a DMatrix<u8>,
    /// Cells with value `>= threshold` are treated as occupied.
    pub threshold: u8,
}

impl<'a> OccupancyGridSpace<'a> {
    /// Convert a world-frame position to a `(row, col)` grid cell.
    ///
    /// Returns `None` if `pos` lies outside the grid boundary.
    pub fn world_to_cell(&self, pos: Vector2<f64>) -> Option<(usize, usize)> {
        let col = ((pos.x - self.origin_x) / self.resolution).floor() as isize;
        let row = ((pos.y - self.origin_y) / self.resolution).floor() as isize;
        if col >= 0 && row >= 0 && (col as usize) < self.ncols && (row as usize) < self.nrows {
            Some((row as usize, col as usize))
        } else {
            None
        }
    }

    /// Return the world-frame centre of cell `(row, col)`.
    pub fn cell_to_world_center(&self, row: usize, col: usize) -> Vector2<f64> {
        Vector2::new(
            self.origin_x + (col as f64 + 0.5) * self.resolution,
            self.origin_y + (row as f64 + 0.5) * self.resolution,
        )
    }

    /// Return `true` if `data[(row, col)] < threshold` (i.e. the cell is free).
    pub fn is_cell_free(&self, row: usize, col: usize) -> bool {
        self.data[(row, col)] < self.threshold
    }

    /// Yield the up-to-8 grid-connected neighbours of `(row, col)`,
    /// excluding cells that fall outside the grid boundary.
    ///
    /// Neighbours are returned in the order: N, S, W, E, NW, NE, SW, SE.
    /// Diagonal moves are included so A\* can find octile-distance-optimal paths.
    pub fn neighbors(&self, row: usize, col: usize) -> impl Iterator<Item = (usize, usize)> {
        const DIRS: [(i32, i32); 8] = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
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

    /// Bresenham line-of-sight check between cells `(r1, c1)` and `(r2, c2)`.
    ///
    /// Traces the integer raster line from the start cell to (but not
    /// including) the end cell, returning `false` as soon as any occupied
    /// cell is encountered. The end cell itself is not checked, matching the
    /// convention expected by the string-pull smoother (the goal cell may be
    /// non-free but the robot still wants a waypoint near it).
    pub fn has_line_of_sight(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> bool {
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

// =========================================================================
// == SearchSpace impl ==
// =========================================================================

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

    /// Clamp `pos` to the outermost cell centres.
    ///
    /// Returns `None` only for a degenerate grid (zero rows or columns).
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
// == Tests ==
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::DMatrix;

    /// Build a uniform grid filled with `fill`.
    fn make_data(nrows: usize, ncols: usize, fill: u8) -> DMatrix<u8> {
        DMatrix::from_element(nrows, ncols, fill)
    }

    /// Construct an [`OccupancyGridSpace`] anchored at the world origin with
    /// 1 m resolution. Threshold defaults to 128 (ROS convention: ≥ 128 = occupied).
    fn grid<'a>(
        data: &'a DMatrix<u8>,
        nrows: usize,
        ncols: usize,
        resolution: f64,
        threshold: u8,
    ) -> OccupancyGridSpace<'a> {
        OccupancyGridSpace {
            origin_x: 0.0,
            origin_y: 0.0,
            resolution,
            nrows,
            ncols,
            data,
            threshold,
        }
    }

    /// Known positions should map to the expected `(row, col)`.
    /// `(0.5, 0.5)` → cell `(0, 0)`; `(3.7, 2.1)` → cell `(2, 3)`.
    #[test]
    fn world_to_cell_basic() {
        let data = make_data(10, 10, 0);
        let space = grid(&data, 10, 10, 1.0, 128);
        assert_eq!(space.world_to_cell(Vector2::new(0.5, 0.5)), Some((0, 0)));
        assert_eq!(space.world_to_cell(Vector2::new(3.7, 2.1)), Some((2, 3)));
    }

    /// Positions outside the grid extent (negative or beyond the far edge)
    /// must return `None`.
    #[test]
    fn world_to_cell_out_of_bounds() {
        let data = make_data(5, 5, 0);
        let space = grid(&data, 5, 5, 1.0, 128);
        assert_eq!(space.world_to_cell(Vector2::new(-0.1, 0.5)), None);
        assert_eq!(space.world_to_cell(Vector2::new(0.5, 5.5)), None);
    }

    /// `cell_to_world_center` followed by `world_to_cell` must be a
    /// fixed-point: the centre of a cell always maps back to the same cell.
    #[test]
    fn cell_to_world_center_roundtrip() {
        let data = make_data(10, 10, 0);
        let space = grid(&data, 10, 10, 1.0, 128);
        let center = space.cell_to_world_center(3, 4);
        let cell = space.world_to_cell(center).unwrap();
        assert_eq!(cell, (3, 4));
    }

    /// Corner cell `(0, 0)` has exactly 3 valid neighbours (clipped by grid boundary).
    #[test]
    fn neighbors_corner() {
        let data = make_data(5, 5, 0);
        let space = grid(&data, 5, 5, 1.0, 128);
        let n: Vec<_> = space.neighbors(0, 0).collect();
        assert_eq!(n.len(), 3);
    }

    /// Interior cell `(2, 2)` has the full 8 neighbours.
    #[test]
    fn neighbors_center() {
        let data = make_data(5, 5, 0);
        let space = grid(&data, 5, 5, 1.0, 128);
        let n: Vec<_> = space.neighbors(2, 2).collect();
        assert_eq!(n.len(), 8);
    }

    /// Threshold is exclusive-lower: value `127 < 128` → free; `128 >= 128` → occupied.
    #[test]
    fn is_cell_free_threshold_boundary() {
        let mut data = DMatrix::from_element(3, 3, 0u8);
        data[(1, 1)] = 127;
        data[(1, 2)] = 128;
        let space = grid(&data, 3, 3, 1.0, 128);
        assert!(space.is_cell_free(1, 1)); // 127 < 128 → free
        assert!(!space.is_cell_free(1, 2)); // 128 >= 128 → occupied
    }

    /// A diagonal across an all-zero grid has clear LOS.
    #[test]
    fn has_line_of_sight_clear() {
        let data = make_data(5, 5, 0);
        let space = grid(&data, 5, 5, 1.0, 128);
        assert!(space.has_line_of_sight(0, 0, 4, 4));
    }

    /// An obstacle at the midpoint of a vertical segment blocks LOS.
    /// Cell `(2, 0)` is occupied; the line from `(0, 0)` to `(4, 0)` must
    /// return `false`.
    #[test]
    fn has_line_of_sight_blocked() {
        let mut data = DMatrix::from_element(5, 5, 0u8);
        data[(2, 0)] = 255;
        let space = grid(&data, 5, 5, 1.0, 128);
        assert!(!space.has_line_of_sight(0, 0, 4, 0));
    }
}
