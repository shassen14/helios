//! The organized range grid a world sensor produces, before it is flattened to a
//! point cloud.
//!
//! [`RangeField`] keeps one range per beam in a `rows × cols` grid, with misses
//! stored as `inf` instead of dropped. That is the information a
//! [`PointCloud`] cannot hold: a miss is a direction with no position, and the
//! grid's adjacency is lost once cells become a flat list of points. Consumers
//! that need either (free-space carving, neighbor-based filters) read the field;
//! everything else takes [`RangeField::to_point_cloud`], the one-way, lossy
//! conversion to the cloud.

use super::DirectionModel;
use crate::interchange::measurement::cloud::{PointCloud, PointColumns, TimeColumn};
use crate::spatial::{conventions::Frame, quantities::Point};

use std::{fmt, marker::PhantomData, sync::Arc};

use nalgebra::{DMatrix, Matrix3xX};

/// An organized grid of ranges in frame `F`, one per beam, addressed by a
/// [`DirectionModel`].
///
/// Each cell holds the range its beam returned, in the direction model's native
/// unit, or `inf` for a miss. The grid's shape is the direction model's
/// [`shape`](DirectionModel::shape), so the two cannot disagree. The field is
/// immutable once built: the crate's range-field builder is the only way to
/// make one.
pub struct RangeField<F: Frame> {
    ranges: DMatrix<f64>,
    direction: DirectionModel,
    range_min: f64,
    range_max: f64,
    timing: Option<ScanTiming>,
    _frame: PhantomData<F>,
}

impl<F: Frame> RangeField<F> {
    /// Assembles a field from already-checked parts, trusting `ranges` to match
    /// the direction model's shape and `timing` to match its axis.
    ///
    /// Crate-private because the builder is the only write path, and it is there
    /// that shape and timing are checked; this constructor deliberately does not
    /// re-check them, so the gate lives in exactly one place.
    pub(crate) fn new(
        ranges: DMatrix<f64>,
        direction: DirectionModel,
        range_min: f64,
        range_max: f64,
        timing: Option<ScanTiming>,
    ) -> Self {
        Self {
            ranges,
            direction,
            range_min,
            range_max,
            timing,
            _frame: PhantomData,
        }
    }

    /// The grid dimensions, as `(rows, cols)`, taken from the direction model.
    pub fn shape(&self) -> (usize, usize) {
        self.direction.shape()
    }

    /// The projection that maps each cell to a beam direction.
    pub fn direction(&self) -> &DirectionModel {
        &self.direction
    }

    /// The shortest range the sensor reports, in meters.
    pub fn range_min(&self) -> f64 {
        self.range_min
    }

    /// The longest range the sensor reports, in meters. A miss carves free space
    /// along its bearing out to this distance.
    pub fn range_max(&self) -> f64 {
        self.range_max
    }

    /// When each row or column was measured, or `None` if the whole grid shares
    /// one instant (a flash capture, or a scan already corrected for motion).
    pub fn timing(&self) -> Option<&ScanTiming> {
        self.timing.as_ref()
    }

    /// The range stored at `(row, col)` — `inf` for a miss — or `None` if the
    /// index lies outside [`shape`](Self::shape).
    pub fn range(&self, row: usize, col: usize) -> Option<f64> {
        self.ranges.get((row, col)).copied()
    }

    /// The point the beam at `(row, col)` struck, in frame `F`, or `None` for a
    /// miss or an index outside [`shape`](Self::shape).
    pub fn deproject(&self, row: usize, col: usize) -> Option<Point<F>> {
        let range = self.range(row, col)?;
        let point = self.direction.deproject(row, col, range)?;
        Some(Point::from_raw(point))
    }

    /// Flattens the grid into a [`PointCloud`], dropping misses.
    ///
    /// Cells are visited column by column, and that order is part of the
    /// contract: for a spinning lidar a column is one azimuth, so the output
    /// is in firing order. A timed field yields a timed cloud, each point
    /// taking its row's or column's offset; an untimed field yields an untimed
    /// cloud.
    ///
    /// Assembles the cloud's columns directly rather than through the cloud
    /// builder. Every kept cell pushes its time offset (when timed) before its
    /// coordinates, and a cell with no offset is skipped whole, so the geometry
    /// and time columns cannot fall out of step. Going through the builder
    /// would only add a length check this loop cannot fail.
    pub fn to_point_cloud(&self) -> PointCloud<F> {
        let (rows, cols) = self.shape();
        let mut coords = Vec::with_capacity(rows * cols * 3);
        let mut times = Vec::with_capacity(rows * cols);

        for col in 0..cols {
            for row in 0..rows {
                let Some(point) = self.deproject(row, col) else {
                    continue;
                };

                if let Some(timing) = &self.timing {
                    let Some(offset) = timing.offset(row, col) else {
                        continue;
                    };
                    times.push(offset);
                }

                coords.extend(point.raw());
            }
        }

        let geometry = PointColumns::from_arc(Arc::new(Matrix3xX::from_column_slice(&coords)));
        let time = self.timing.is_some().then(|| TimeColumn::from_vec(times));

        PointCloud::from_columns(geometry, (), time)
    }
}

// Hand-written rather than derived: `#[derive(Clone)]` would demand `F: Clone`
// even though `F` lives only inside a `PhantomData`, and that impl is then
// invisible to generic code that knows only `F: Frame`.
impl<F: Frame> Clone for RangeField<F> {
    fn clone(&self) -> Self {
        Self {
            ranges: self.ranges.clone(),
            direction: self.direction.clone(),
            range_min: self.range_min,
            range_max: self.range_max,
            timing: self.timing.clone(),
            _frame: PhantomData,
        }
    }
}

// Hand-written for the same reason as `Clone`, plus one of its own: a derive
// would print every range in the grid — tens of thousands of numbers for a
// multi-ring lidar. This prints a summary instead.
impl<F: Frame> fmt::Debug for RangeField<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RangeField")
            .field("shape", &self.shape())
            .field("range_min", &self.range_min)
            .field("range_max", &self.range_max)
            .field("timed", &self.timing.is_some())
            .finish()
    }
}

/// When each part of a grid was measured, as nanosecond offsets from the
/// reading's timestamp.
///
/// Real sensors vary their timing along one axis only, so offsets are stored
/// per row or per column, never per cell.
#[derive(Debug, Clone, PartialEq)]
pub enum ScanTiming {
    /// One offset per row, as in a rolling-shutter camera.
    PerRow(Vec<i32>),
    /// One offset per column, as in a spinning lidar: each azimuth fires at its
    /// own moment, and every ring in that column fires together.
    PerColumn(Vec<i32>),
}

impl ScanTiming {
    /// The offset for cell `(row, col)`, or `None` if its row or column has no
    /// entry.
    pub fn offset(&self, row: usize, col: usize) -> Option<i32> {
        match self {
            Self::PerRow(offsets) => offsets.get(row).copied(),
            Self::PerColumn(offsets) => offsets.get(col).copied(),
        }
    }

    /// The offsets themselves, whichever axis they run along.
    pub fn offsets(&self) -> &[i32] {
        match self {
            Self::PerRow(offsets) | Self::PerColumn(offsets) => offsets,
        }
    }

    /// How many offsets a `rows × cols` grid needs: one per row for
    /// [`PerRow`](Self::PerRow), one per column for
    /// [`PerColumn`](Self::PerColumn).
    pub fn expected_len(&self, rows: usize, cols: usize) -> usize {
        match self {
            Self::PerRow(_) => rows,
            Self::PerColumn(_) => cols,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::interchange::measurement::range_field::SphericalAngular;
    use crate::spatial::conventions::Flu;

    const RANGE_MIN: f64 = 0.1;
    const RANGE_MAX: f64 = 100.0;

    /// Tolerance for comparing points built from trig.
    const TOLERANCE: f64 = 1e-12;

    /// A 2-ring × 3-azimuth grid with one miss in each ring:
    ///
    /// ```text
    ///          col 0   col 1   col 2
    /// row 0     1.0     inf     3.0
    /// row 1     4.0     5.0     inf
    /// ```
    ///
    /// Walked column by column, the hits are `(0,0) (1,0) (1,1) (0,2)`.
    fn field_with(timing: Option<ScanTiming>) -> RangeField<Flu> {
        let direction = DirectionModel::SphericalAngular(
            SphericalAngular::new(vec![0.0, 0.2], 0.0, 0.5, 3).expect("valid geometry"),
        );
        let ranges =
            DMatrix::from_row_slice(2, 3, &[1.0, f64::INFINITY, 3.0, 4.0, 5.0, f64::INFINITY]);
        RangeField::new(ranges, direction, RANGE_MIN, RANGE_MAX, timing)
    }

    /// The hit cells of [`field_with`], in column-major order.
    const HITS_COLUMN_MAJOR: [(usize, usize); 4] = [(0, 0), (1, 0), (1, 1), (0, 2)];

    fn assert_close(actual: Point<Flu>, expected: Point<Flu>) {
        assert!(
            (actual.raw() - expected.raw()).norm() < TOLERANCE,
            "expected {expected:?}, got {actual:?}"
        );
    }

    #[test]
    fn shape_comes_from_the_direction_model() {
        let field = field_with(None);
        assert_eq!(field.shape(), (2, 3));
        assert_eq!(field.shape(), field.direction().shape());
    }

    #[test]
    fn range_reads_the_cell_and_none_out_of_bounds() {
        let field = field_with(None);
        assert_eq!(field.range(1, 1), Some(5.0));
        assert_eq!(field.range(0, 1), Some(f64::INFINITY));
        assert_eq!(field.range(2, 0), None);
        assert_eq!(field.range(0, 3), None);
    }

    #[test]
    fn deproject_tags_the_direction_models_point_with_the_frame() {
        let field = field_with(None);
        let raw = field.direction().deproject(1, 1, 5.0).unwrap();
        assert_close(field.deproject(1, 1).unwrap(), Point::from_raw(raw));
    }

    #[test]
    fn deproject_yields_none_for_a_miss_or_out_of_bounds() {
        let field = field_with(None);
        assert!(field.deproject(0, 1).is_none());
        assert!(field.deproject(2, 0).is_none());
    }

    #[test]
    fn to_point_cloud_drops_misses_and_walks_column_major() {
        let field = field_with(None);
        let cloud = field.to_point_cloud();

        assert_eq!(cloud.len(), HITS_COLUMN_MAJOR.len());
        for (i, &(row, col)) in HITS_COLUMN_MAJOR.iter().enumerate() {
            assert_close(cloud.point(i), field.deproject(row, col).unwrap());
        }
    }

    #[test]
    fn an_untimed_field_yields_an_untimed_cloud() {
        let cloud = field_with(None).to_point_cloud();
        assert!(cloud.time().is_none());
    }

    #[test]
    fn per_column_timing_gives_each_point_its_columns_offset() {
        let cloud = field_with(Some(ScanTiming::PerColumn(vec![0, 100, 200]))).to_point_cloud();
        let times = cloud.time().expect("timed field yields a timed cloud");
        assert_eq!(times.as_slice(), &[0, 0, 100, 200]);
    }

    #[test]
    fn per_row_timing_gives_each_point_its_rows_offset() {
        let cloud = field_with(Some(ScanTiming::PerRow(vec![10, 20]))).to_point_cloud();
        let times = cloud.time().expect("timed field yields a timed cloud");
        assert_eq!(times.as_slice(), &[10, 20, 20, 10]);
    }

    /// The builder rejects a timing vector of the wrong length, so this cannot
    /// arise from a built field; it checks that `to_point_cloud` still keeps the
    /// geometry and time columns the same length if it ever does, since the
    /// cloud is assembled without the cloud builder's length check.
    #[test]
    fn a_cell_with_no_offset_is_skipped_whole() {
        let cloud = field_with(Some(ScanTiming::PerColumn(vec![0, 100]))).to_point_cloud();
        let times = cloud.time().expect("timed field yields a timed cloud");

        assert_eq!(
            cloud.len(),
            3,
            "the column-2 hit has no offset and is skipped"
        );
        assert_eq!(times.len(), cloud.len());
        assert_eq!(times.as_slice(), &[0, 0, 100]);
    }

    #[test]
    fn an_all_miss_field_yields_an_empty_cloud() {
        let direction = DirectionModel::SphericalAngular(
            SphericalAngular::new(vec![0.0], 0.0, 0.5, 4).expect("valid geometry"),
        );
        let ranges = DMatrix::from_element(1, 4, f64::INFINITY);
        let field: RangeField<Flu> = RangeField::new(
            ranges,
            direction,
            RANGE_MIN,
            RANGE_MAX,
            Some(ScanTiming::PerColumn(vec![0, 1, 2, 3])),
        );

        let cloud = field.to_point_cloud();
        assert!(cloud.is_empty());
        assert_eq!(cloud.time().map(TimeColumn::len), Some(0));
    }

    #[test]
    fn debug_prints_a_summary_not_the_grid() {
        let printed = format!(
            "{:?}",
            field_with(Some(ScanTiming::PerColumn(vec![0, 1, 2])))
        );
        assert!(printed.contains("shape: (2, 3)"), "{printed}");
        assert!(printed.contains("timed: true"), "{printed}");
        assert!(!printed.contains("inf"), "{printed}");
    }

    #[test]
    fn scan_timing_offset_picks_the_axis_and_none_past_the_end() {
        let per_row = ScanTiming::PerRow(vec![10, 20]);
        let per_column = ScanTiming::PerColumn(vec![0, 100, 200]);

        assert_eq!(per_row.offset(1, 2), Some(20));
        assert_eq!(per_column.offset(1, 2), Some(200));
        assert_eq!(per_row.offset(2, 0), None);
        assert_eq!(per_column.offset(0, 3), None);
    }

    #[test]
    fn scan_timing_expects_one_offset_per_entry_on_its_axis() {
        let per_row = ScanTiming::PerRow(vec![10, 20]);
        let per_column = ScanTiming::PerColumn(vec![0, 100, 200]);

        assert_eq!(per_row.expected_len(2, 3), 2);
        assert_eq!(per_column.expected_len(2, 3), 3);
        assert_eq!(per_row.offsets(), &[10, 20]);
        assert_eq!(per_column.offsets(), &[0, 100, 200]);
    }
}
