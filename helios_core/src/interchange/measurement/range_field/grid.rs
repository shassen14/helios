//! Per-cell attributes carried alongside a [`RangeField`]'s ranges.
//!
//! A sensor often measures more than distance at each beam (return intensity,
//! ambient light, color). Before flattening, those values live here in the
//! same `rows × cols` shape as the ranges, filled by index and kept for every
//! cell, misses included. [`GridAttributes::gather`] turns the cells a
//! flattening keeps into the cloud's attribute columns, and is the place where
//! an attribute the grid gets for free from its position (a lidar's ring is its
//! row) becomes a stored column in the cloud.
//!
//! [`RangeField`]: super::RangeField

use crate::interchange::measurement::attribute::canonical::{INTENSITY, RING};
use crate::interchange::measurement::attribute::column::AttributeColumn;
use crate::interchange::measurement::attribute::schema::AttributeSchema;
use crate::interchange::measurement::attribute::table::AttributeTable;

use nalgebra::DMatrix;

/// A bundle of attribute grids, each the same shape as a range field's ranges.
///
/// Grids are not masked by the ranges: a cell whose range is a miss keeps
/// whatever attributes the sensor reported for it, and only the flattening to
/// a cloud drops them.
pub trait GridAttributes: Clone + Send + Sync + 'static {
    /// What a producer writes into one cell.
    type Cell;

    /// A `rows × cols` bundle with every cell at its blank value, or `None` if
    /// this bundle cannot represent a grid of that shape.
    fn blank(rows: usize, cols: usize) -> Option<Self>;

    /// Writes `cell` into `(row, col)`. An index outside the grid is ignored:
    /// the range-field builder checks bounds before delegating here, so that
    /// branch exists only to keep this method free of a panic path.
    fn set(&mut self, row: usize, col: usize, cell: Self::Cell);

    /// The cloud's attribute columns for the points that `cells` become: one
    /// value per `(row, col)`, in the given order, with the columns in the
    /// table's schema order.
    fn gather(&self, cells: &[(usize, usize)]) -> AttributeTable;
}

/// No attributes: a range field that carries ranges only.
impl GridAttributes for () {
    type Cell = ();

    fn blank(_rows: usize, _cols: usize) -> Option<Self> {
        Some(())
    }

    fn set(&mut self, _row: usize, _col: usize, _cell: Self::Cell) {}

    fn gather(&self, _cells: &[(usize, usize)]) -> AttributeTable {
        AttributeTable::default()
    }
}

/// The grid bundle for a spinning lidar: return intensity per cell. It
/// flattens into an [`INTENSITY`] column plus a [`RING`] column holding each
/// cell's row index.
#[derive(Clone)]
pub struct LidarGrids {
    /// The cloud columns [`gather`](GridAttributes::gather) builds, in the
    /// order it builds them.
    schema: AttributeSchema,
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

    /// Rejects a grid whose last row index does not fit the cloud's `u16` ring
    /// column, so [`gather`](GridAttributes::gather) never has to narrow a row
    /// that would not fit.
    fn blank(rows: usize, cols: usize) -> Option<Self> {
        let last_row = rows.saturating_sub(1);
        if u16::try_from(last_row).is_err() {
            return None;
        }

        // Two distinct names, so this cannot fail; `?` keeps it panic-free.
        let schema = AttributeSchema::new([INTENSITY.descriptor(), RING.descriptor()]).ok()?;

        Some(Self {
            schema,
            intensity: DMatrix::from_element(rows, cols, Self::NO_INTENSITY),
        })
    }

    fn set(&mut self, row: usize, col: usize, cell: Self::Cell) {
        if let Some(slot) = self.intensity.get_mut((row, col)) {
            *slot = cell.intensity;
        }
    }

    fn gather(&self, cells: &[(usize, usize)]) -> AttributeTable {
        let mut intensities = Vec::with_capacity(cells.len());
        let mut rings = Vec::with_capacity(cells.len());

        for &(row, col) in cells {
            // Both fallbacks are unreachable: the field only passes cells
            // inside its shape, and `blank` rejected any row too large for a
            // ring. Each is a value no real cell holds, never a plausible one.
            intensities.push(self.intensity(row, col).unwrap_or(Self::NO_INTENSITY));
            rings.push(u16::try_from(row).unwrap_or(u16::MAX));
        }

        // Must follow `self.schema`'s order: the table pairs columns with
        // attributes by position.
        let columns = vec![
            AttributeColumn::F32(intensities.into()),
            AttributeColumn::U16(rings.into()),
        ];

        AttributeTable::new(self.schema.clone(), columns)
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
    fn unit_bundle_gathers_an_empty_table() {
        assert!(().gather(&[(0, 0), (1, 2)]).schema().is_empty());
    }

    /// Reading back through the keys, not by position, is what catches a
    /// column order or element type that drifted from the schema: either
    /// makes `get` return `None`.
    #[test]
    fn gather_carries_each_cells_intensity_and_its_row_as_ring() {
        let mut grids = LidarGrids::blank(3, 2).expect("three rings fit a ring index");
        grids.set(2, 1, LidarCell { intensity: 0.4 });
        grids.set(0, 0, LidarCell { intensity: 0.8 });

        let table = grids.gather(&[(2, 1), (0, 0)]);

        assert_eq!(table.get(INTENSITY), Some([0.4, 0.8].as_slice()));
        assert_eq!(table.get(RING), Some([2, 0].as_slice()));
    }

    #[test]
    fn gather_keeps_the_given_order_and_repeats() {
        let mut grids = LidarGrids::blank(2, 2).expect("two rings fit a ring index");
        grids.set(1, 0, LidarCell { intensity: 0.5 });

        let table = grids.gather(&[(1, 0), (0, 1), (1, 0)]);

        assert_eq!(table.get(RING), Some([1, 0, 1].as_slice()));
        let intensity = table.get(INTENSITY).expect("lidar table carries intensity");
        assert_eq!(intensity[0], 0.5);
        assert!(intensity[1].is_nan(), "an unwritten cell stays blank");
        assert_eq!(intensity[2], 0.5);
    }

    #[test]
    fn gather_of_no_cells_yields_empty_columns_under_the_full_schema() {
        let grids = LidarGrids::blank(2, 3).expect("two rings fit a ring index");
        let table = grids.gather(&[]);

        assert_eq!(table.schema().len(), 2);
        assert_eq!(table.get(INTENSITY), Some([].as_slice()));
        assert_eq!(table.get(RING), Some([].as_slice()));
    }

    /// Row `u16::MAX - 1` rather than the last row: the unreachable fallback
    /// also yields `u16::MAX`, so only a row below it shows the conversion.
    #[test]
    fn gather_keeps_high_ring_indices_exact() {
        let grids = LidarGrids::blank(MAX_RINGS, 1).expect("largest grid a ring can index");
        let high_row = usize::from(u16::MAX) - 1;

        let table = grids.gather(&[(high_row, 0)]);
        assert_eq!(table.get(RING), Some([u16::MAX - 1].as_slice()));
    }
}
