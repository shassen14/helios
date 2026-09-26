//! Per-cell attributes carried alongside a [`RangeField`]'s ranges.
//!
//! A sensor often measures more than distance at each beam (return intensity,
//! ambient light, color). Before flattening, those values live here in the
//! same `rows × cols` shape as the ranges, filled by index and kept for every
//! cell, misses included. [`GridAttributes::cloud_row`] converts one cell into
//! one row of the matching point-cloud bundle, and is the place where an
//! attribute the grid gets for free from its position (a lidar's ring is its
//! row) becomes a stored column in the cloud.
//!
//! [`RangeField`]: super::RangeField

use crate::interchange::measurement::cloud::{Attributes, LidarAttrs, LidarColumns};

use nalgebra::DMatrix;

/// A bundle of attribute grids, each the same shape as a range field's ranges,
/// paired with the point-cloud bundle it flattens into.
///
/// Grids are not masked by the ranges: a cell whose range is a miss keeps
/// whatever attributes the sensor reported for it, and only the flattening to
/// a cloud drops them.
pub trait GridAttributes: Clone + Send + Sync + 'static {
    /// What a producer writes into one cell.
    type Cell;
    /// The point-cloud bundle this grid flattens into.
    type Cloud: Attributes;

    /// A `rows × cols` bundle with every cell at its blank value, or `None` if
    /// this bundle cannot represent a grid of that shape.
    fn blank(rows: usize, cols: usize) -> Option<Self>;

    /// Writes `cell` into `(row, col)`. An index outside the grid is ignored:
    /// the range-field builder checks bounds before delegating here, so that
    /// branch exists only to keep this method free of a panic path.
    fn set(&mut self, row: usize, col: usize, cell: Self::Cell);

    /// The cloud row for the point that cell `(row, col)` becomes.
    fn cloud_row(&self, row: usize, col: usize) -> <Self::Cloud as Attributes>::Row;
}

/// No attributes: a range field that carries ranges only.
impl GridAttributes for () {
    type Cell = ();
    type Cloud = ();

    fn blank(_rows: usize, _cols: usize) -> Option<Self> {
        Some(())
    }

    fn set(&mut self, _row: usize, _col: usize, _cell: Self::Cell) {}

    fn cloud_row(&self, _row: usize, _col: usize) -> <Self::Cloud as Attributes>::Row {}
}

/// The grid bundle for a spinning lidar: return intensity per cell. It
/// flattens into [`LidarColumns`], whose ring column is each cell's row index.
#[derive(Clone)]
pub struct LidarGrids {
    intensity: DMatrix<f32>,
}

impl LidarGrids {
    /// The blank intensity: the sensor reported none for this cell. Not zero,
    /// because zero is a real reflectance.
    pub const NO_INTENSITY: f32 = f32::NAN;

    /// The intensity at `(row, col)`, or `None` if the index lies outside the
    /// grid.
    pub fn intensity(&self, row: usize, col: usize) -> Option<f32> {
        self.intensity.get((row, col)).copied()
    }
}

impl GridAttributes for LidarGrids {
    type Cell = LidarCell;
    type Cloud = LidarColumns;

    /// Rejects a grid whose last row index does not fit the cloud's `u16` ring
    /// column, so [`cloud_row`](GridAttributes::cloud_row) never has to narrow
    /// a row that would not fit.
    fn blank(rows: usize, cols: usize) -> Option<Self> {
        let last_row = rows.saturating_sub(1);
        if u16::try_from(last_row).is_err() {
            return None;
        }

        Some(Self {
            intensity: DMatrix::from_element(rows, cols, Self::NO_INTENSITY),
        })
    }

    fn set(&mut self, row: usize, col: usize, cell: Self::Cell) {
        if let Some(slot) = self.intensity.get_mut((row, col)) {
            *slot = cell.intensity;
        }
    }

    fn cloud_row(&self, row: usize, col: usize) -> <Self::Cloud as Attributes>::Row {
        // Both fallbacks are unreachable: the field only asks for cells inside
        // its shape, and `blank` rejected any row too large for a ring.
        LidarAttrs {
            intensity: self.intensity(row, col).unwrap_or(Self::NO_INTENSITY),
            ring: u16::try_from(row).unwrap_or(u16::MAX),
        }
    }
}

/// One lidar cell's attributes, as a producer writes them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarCell {
    pub intensity: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The most rows a lidar grid can have: one per `u16` ring index.
    // `as` because `From` is not callable in a `const`; widening is lossless.
    const MAX_RINGS: usize = u16::MAX as usize + 1;

    #[test]
    fn unit_bundle_accepts_any_shape() {
        assert_eq!(<() as GridAttributes>::blank(MAX_RINGS + 1, 1), Some(()));
    }

    #[test]
    fn a_blank_lidar_grid_reports_no_intensity_anywhere() {
        let grids = LidarGrids::blank(2, 3).expect("two rings fit a ring index");
        for row in 0..2 {
            for col in 0..3 {
                let intensity = grids.intensity(row, col).expect("inside the grid");
                assert!(intensity.is_nan(), "({row}, {col}) was {intensity}");
            }
        }
    }

    #[test]
    fn blank_rejects_only_a_grid_whose_last_row_overflows_a_ring() {
        assert!(LidarGrids::blank(MAX_RINGS, 1).is_some());
        assert!(LidarGrids::blank(MAX_RINGS + 1, 1).is_none());
    }

    #[test]
    fn blank_accepts_an_empty_grid() {
        assert!(LidarGrids::blank(0, 0).is_some());
    }

    #[test]
    fn set_writes_one_cell_and_ignores_an_index_outside_the_grid() {
        let mut grids = LidarGrids::blank(2, 3).expect("two rings fit a ring index");
        grids.set(1, 2, LidarCell { intensity: 0.7 });
        grids.set(2, 0, LidarCell { intensity: 0.9 });
        grids.set(0, 3, LidarCell { intensity: 0.9 });

        assert_eq!(grids.intensity(1, 2), Some(0.7));
        assert!(grids.intensity(0, 0).is_some_and(f32::is_nan));
        assert_eq!(grids.intensity(2, 0), None);
    }

    #[test]
    fn cloud_row_carries_the_intensity_and_the_row_as_ring() {
        let mut grids = LidarGrids::blank(3, 2).expect("three rings fit a ring index");
        grids.set(2, 1, LidarCell { intensity: 0.4 });

        let row = grids.cloud_row(2, 1);
        assert_eq!(row.intensity, 0.4);
        assert_eq!(row.ring, 2);
    }

    /// Row `u16::MAX - 1` rather than the last row: the unreachable fallback
    /// also yields `u16::MAX`, so only a row below it shows the conversion.
    #[test]
    fn cloud_row_keeps_high_ring_indices_exact() {
        let grids = LidarGrids::blank(MAX_RINGS, 1).expect("largest grid a ring can index");
        let high_row = usize::from(u16::MAX) - 1;
        assert_eq!(grids.cloud_row(high_row, 0).ring, u16::MAX - 1);
    }
}
