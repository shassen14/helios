//! The one write path into a [`RangeField`].
//!
//! A grid's size is known before any data arrives, and both hosts address cells
//! by index — the sim by ray id, a hardware driver as packets land. So the
//! builder is sized from its [`DirectionModel`] up front, starts with every cell
//! at [`NOTHING_RETURNED`], and is filled in place by
//! [`set`](RangeFieldBuilder::set) rather than appended to. Per-cell attributes are filled the same way, through
//! [`set_attributes`](RangeFieldBuilder::set_attributes). Every check happens at
//! the call that could get it wrong, which leaves
//! [`finalize`](RangeFieldBuilder::finalize) nothing to reject.

use super::{DirectionModel, GridAttributes, RangeField, ScanTiming, NOTHING_RETURNED};
use crate::spatial::conventions::Frame;

use std::{fmt, marker::PhantomData};

use nalgebra::DMatrix;

/// Fills a [`RangeField`] in frame `F` cell by cell.
///
/// Holds the same parts as the field it builds. A producer whose geometry,
/// limits, and timing are fixed can build one blank builder at setup, where any
/// rejection is a configuration error, and clone it for every scan.
///
/// `G` is the field's attribute bundle, filled separately from the ranges; a
/// builder of ranges alone uses `()`.
pub struct RangeFieldBuilder<F: Frame, G: GridAttributes = ()> {
    ranges: DMatrix<f64>,
    attributes: G,
    direction: DirectionModel,
    range_min: f64,
    range_max: f64,
    timing: Option<ScanTiming>,
    _frame: PhantomData<F>,
}

impl<F: Frame, G: GridAttributes> RangeFieldBuilder<F, G> {
    /// Starts a grid shaped by `direction`, with every cell at
    /// [`NOTHING_RETURNED`] and every attribute at its blank value. A producer
    /// then writes only the beams that came back, plus
    /// [`NO_INFORMATION`](super::NO_INFORMATION) for any beam it cannot vouch
    /// for.
    ///
    /// Rejects range limits that are non-finite, negative, or leave no valid
    /// span (`range_min >= range_max`); consumers carve free space out to
    /// `range_max`, so it must be a real distance. Also rejects a shape the
    /// attribute bundle cannot represent, such as a lidar grid with more rows
    /// than its ring index can count.
    pub fn new(
        direction: DirectionModel,
        range_min: f64,
        range_max: f64,
    ) -> Result<Self, RangeFieldBuildError> {
        if !range_min.is_finite()
            || !range_max.is_finite()
            || range_min < 0.0
            || range_min >= range_max
        {
            return Err(RangeFieldBuildError::InvalidRangeLimits {
                min: range_min,
                max: range_max,
            });
        }

        let (rows, cols) = direction.shape();
        let Some(blank) = G::blank(rows, cols) else {
            return Err(RangeFieldBuildError::AttributeGridRejected {
                shape: (rows, cols),
            });
        };

        Ok(Self {
            ranges: DMatrix::from_element(rows, cols, NOTHING_RETURNED),
            attributes: blank,
            direction,
            range_min,
            range_max,
            timing: None,
            _frame: PhantomData,
        })
    }

    /// Attaches per-row or per-column time offsets. Leave it uncalled for a
    /// grid measured at a single instant.
    ///
    /// Rejects `timing` unless it holds exactly one offset per row or column of
    /// the grid, so a built field never has a cell without an offset.
    pub fn with_timing(mut self, timing: ScanTiming) -> Result<Self, RangeFieldBuildError> {
        let (rows, cols) = self.direction.shape();
        let expected = timing.expected_len(rows, cols);
        let got = timing.offsets().len();
        if got != expected {
            return Err(RangeFieldBuildError::TimingLengthMismatch { expected, got });
        }

        self.timing = Some(timing);
        Ok(self)
    }

    /// Writes `range` into cell `(row, col)`, replacing whatever was there.
    ///
    /// The value is stored as given: the producer decides which sentinel a
    /// return it would not report becomes, since only it knows why the return
    /// failed. An index outside the grid is an
    /// error rather than a panic, because the index comes from the host; the
    /// caller skips that cell and keeps filling.
    pub fn set(&mut self, row: usize, col: usize, range: f64) -> Result<(), RangeFieldBuildError> {
        self.check_cell(row, col)?;
        self.ranges[(row, col)] = range;
        Ok(())
    }

    /// Writes `cell` into the attributes at `(row, col)`, replacing whatever
    /// was there.
    ///
    /// Independent of [`set`](Self::set): either may be written first, and
    /// attributes are kept whatever the cell's range, since some sensors report
    /// them for misses too (ambient light, for one). An index outside the grid
    /// is an error rather than a panic, for the same reason as `set`.
    pub fn set_attributes(
        &mut self,
        row: usize,
        col: usize,
        cell: G::Cell,
    ) -> Result<(), RangeFieldBuildError> {
        self.check_cell(row, col)?;
        self.attributes.set(row, col, cell);
        Ok(())
    }

    /// Freezes the grid into a [`RangeField`]. Cannot fail: shape is fixed at
    /// construction and timing was checked when it was attached.
    pub fn finalize(self) -> RangeField<F, G> {
        RangeField::new(
            self.ranges,
            self.attributes,
            self.direction,
            self.range_min,
            self.range_max,
            self.timing,
        )
    }

    /// Whether `(row, col)` lies inside the grid. The ranges carry the shape
    /// for the whole builder, since the attribute bundle is the same shape but
    /// opaque to generic code.
    fn check_cell(&self, row: usize, col: usize) -> Result<(), RangeFieldBuildError> {
        let (rows, cols) = self.ranges.shape();
        if row < rows && col < cols {
            return Ok(());
        }
        Err(RangeFieldBuildError::OutOfBounds {
            row,
            col,
            shape: (rows, cols),
        })
    }
}

// Hand-written rather than derived: `#[derive(Clone)]` would demand `F: Clone`
// even though `F` lives only inside a `PhantomData`, and that impl is then
// invisible to generic code that knows only `F: Frame`.
impl<F: Frame, G: GridAttributes> Clone for RangeFieldBuilder<F, G> {
    fn clone(&self) -> Self {
        Self {
            ranges: self.ranges.clone(),
            attributes: self.attributes.clone(),
            direction: self.direction.clone(),
            range_min: self.range_min,
            range_max: self.range_max,
            timing: self.timing.clone(),
            _frame: PhantomData,
        }
    }
}

// Hand-written for the same reason as `Clone`, and to print a summary rather
// than every cell of the grid.
impl<F: Frame, G: GridAttributes> fmt::Debug for RangeFieldBuilder<F, G> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RangeFieldBuilder")
            .field("shape", &self.ranges.shape())
            .field("range_min", &self.range_min)
            .field("range_max", &self.range_max)
            .field("timed", &self.timing.is_some())
            .finish()
    }
}

/// Why a [`RangeFieldBuilder`] rejected an input.
#[derive(Debug, Clone, PartialEq)]
pub enum RangeFieldBuildError {
    /// The range limits were non-finite, negative, or `min >= max`.
    InvalidRangeLimits { min: f64, max: f64 },
    /// A cell index lay outside the grid's `(rows, cols)` shape.
    OutOfBounds {
        row: usize,
        col: usize,
        shape: (usize, usize),
    },
    /// The timing held `got` offsets where its axis has `expected` entries.
    TimingLengthMismatch { expected: usize, got: usize },
    /// The attribute bundle cannot represent a grid of this `(rows, cols)`
    /// shape.
    AttributeGridRejected { shape: (usize, usize) },
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::interchange::measurement::range_field::{LidarCell, LidarGrids, SphericalAngular};
    use crate::spatial::conventions::Flu;

    const RANGE_MIN: f64 = 0.1;
    const RANGE_MAX: f64 = 100.0;

    /// A 2-ring × 3-azimuth direction model.
    fn direction() -> DirectionModel {
        DirectionModel::SphericalAngular(
            SphericalAngular::new(vec![0.0, 0.2], 0.0, 0.5, 3).expect("valid geometry"),
        )
    }

    fn builder() -> RangeFieldBuilder<Flu> {
        RangeFieldBuilder::new(direction(), RANGE_MIN, RANGE_MAX).expect("valid limits")
    }

    fn lidar_builder() -> RangeFieldBuilder<Flu, LidarGrids> {
        RangeFieldBuilder::new(direction(), RANGE_MIN, RANGE_MAX).expect("valid limits")
    }

    #[test]
    fn a_fresh_builder_is_all_misses_in_the_direction_models_shape() {
        let field = builder().finalize();

        assert_eq!(field.shape(), (2, 3));
        for row in 0..2 {
            for col in 0..3 {
                assert_eq!(field.range(row, col), Some(NOTHING_RETURNED));
            }
        }
        assert!(field.to_point_cloud().is_empty());
        assert!(field.timing().is_none());
    }

    #[test]
    fn new_rejects_invalid_range_limits() {
        let cases = [
            (-0.1, RANGE_MAX),
            (RANGE_MIN, f64::INFINITY),
            (f64::NAN, RANGE_MAX),
            (RANGE_MAX, RANGE_MAX),
            (RANGE_MAX, RANGE_MIN),
        ];
        for (min, max) in cases {
            assert!(
                matches!(
                    RangeFieldBuilder::<Flu>::new(direction(), min, max),
                    Err(RangeFieldBuildError::InvalidRangeLimits { .. })
                ),
                "min {min}, max {max} should be rejected"
            );
        }
    }

    #[test]
    fn new_accepts_a_zero_range_min() {
        assert!(RangeFieldBuilder::<Flu>::new(direction(), 0.0, RANGE_MAX).is_ok());
    }

    #[test]
    fn finalize_carries_the_limits_and_direction_through() {
        let field = builder().finalize();
        assert_eq!(field.range_min(), RANGE_MIN);
        assert_eq!(field.range_max(), RANGE_MAX);
        assert_eq!(field.direction(), &direction());
    }

    #[test]
    fn set_writes_the_cell_and_leaves_the_rest_missed() {
        let mut builder = builder();
        builder.set(1, 2, 7.5).expect("in bounds");
        let field = builder.finalize();

        assert_eq!(field.range(1, 2), Some(7.5));
        assert_eq!(field.range(0, 2), Some(NOTHING_RETURNED));
    }

    #[test]
    fn a_later_set_overwrites_an_earlier_one() {
        let mut builder = builder();
        builder.set(0, 0, 1.0).expect("in bounds");
        builder.set(0, 0, 2.0).expect("in bounds");
        assert_eq!(builder.finalize().range(0, 0), Some(2.0));
    }

    #[test]
    fn set_out_of_bounds_is_an_error_and_leaves_the_grid_untouched() {
        let mut builder = builder();

        assert_eq!(
            builder.set(2, 0, 1.0),
            Err(RangeFieldBuildError::OutOfBounds {
                row: 2,
                col: 0,
                shape: (2, 3)
            })
        );
        assert_eq!(
            builder.set(0, 3, 1.0),
            Err(RangeFieldBuildError::OutOfBounds {
                row: 0,
                col: 3,
                shape: (2, 3)
            })
        );
        assert!(builder.finalize().to_point_cloud().is_empty());
    }

    #[test]
    fn with_timing_accepts_one_offset_per_entry_on_its_axis() {
        let per_row = ScanTiming::PerRow(vec![10, 20]);
        let per_column = ScanTiming::PerColumn(vec![0, 100, 200]);

        let field = builder()
            .with_timing(per_row.clone())
            .expect("fits")
            .finalize();
        assert_eq!(field.timing(), Some(&per_row));

        let field = builder()
            .with_timing(per_column.clone())
            .expect("fits")
            .finalize();
        assert_eq!(field.timing(), Some(&per_column));
    }

    #[test]
    fn with_timing_rejects_a_length_that_does_not_match_its_axis() {
        // Three offsets fit the columns but not the two rows, and vice versa.
        assert_eq!(
            builder()
                .with_timing(ScanTiming::PerRow(vec![0, 1, 2]))
                .err(),
            Some(RangeFieldBuildError::TimingLengthMismatch {
                expected: 2,
                got: 3
            })
        );
        assert_eq!(
            builder()
                .with_timing(ScanTiming::PerColumn(vec![0, 1]))
                .err(),
            Some(RangeFieldBuildError::TimingLengthMismatch {
                expected: 3,
                got: 2
            })
        );
    }

    #[test]
    fn a_clone_is_filled_independently_of_its_blank() {
        let blank = builder()
            .with_timing(ScanTiming::PerColumn(vec![0, 100, 200]))
            .expect("fits");

        let mut scan = blank.clone();
        scan.set(0, 0, 4.0).expect("in bounds");
        let filled = scan.finalize();
        let untouched = blank.finalize();

        assert_eq!(filled.range(0, 0), Some(4.0));
        assert_eq!(untouched.range(0, 0), Some(NOTHING_RETURNED));
        assert_eq!(filled.timing(), untouched.timing());
    }

    #[test]
    fn a_fresh_lidar_builder_has_blank_intensity_everywhere() {
        let field = lidar_builder().finalize();
        for row in 0..2 {
            for col in 0..3 {
                assert!(field
                    .attributes()
                    .intensity(row, col)
                    .is_some_and(f32::is_nan));
            }
        }
    }

    #[test]
    fn new_rejects_a_shape_the_attribute_bundle_cannot_represent() {
        // One ring more than a `u16` ring index can count.
        let too_many_rings = vec![0.0; u16::MAX as usize + 2];
        let direction = DirectionModel::SphericalAngular(
            SphericalAngular::new(too_many_rings, 0.0, 0.5, 1).expect("valid geometry"),
        );
        let (rows, cols) = direction.shape();

        assert_eq!(
            RangeFieldBuilder::<Flu, LidarGrids>::new(direction.clone(), RANGE_MIN, RANGE_MAX)
                .err(),
            Some(RangeFieldBuildError::AttributeGridRejected {
                shape: (rows, cols)
            })
        );
        assert!(
            RangeFieldBuilder::<Flu>::new(direction, RANGE_MIN, RANGE_MAX).is_ok(),
            "the shape itself is fine without attributes"
        );
    }

    #[test]
    fn set_attributes_writes_the_cell_and_leaves_the_rest_blank() {
        let mut builder = lidar_builder();
        builder
            .set_attributes(1, 2, LidarCell { intensity: 0.7 })
            .expect("in bounds");
        let field = builder.finalize();

        assert_eq!(field.attributes().intensity(1, 2), Some(0.7));
        assert!(field.attributes().intensity(0, 2).is_some_and(f32::is_nan));
    }

    #[test]
    fn set_attributes_out_of_bounds_is_an_error_and_leaves_the_grid_untouched() {
        let mut builder = lidar_builder();

        assert_eq!(
            builder.set_attributes(2, 0, LidarCell { intensity: 0.7 }),
            Err(RangeFieldBuildError::OutOfBounds {
                row: 2,
                col: 0,
                shape: (2, 3)
            })
        );
        assert_eq!(
            builder.set_attributes(0, 3, LidarCell { intensity: 0.7 }),
            Err(RangeFieldBuildError::OutOfBounds {
                row: 0,
                col: 3,
                shape: (2, 3)
            })
        );

        let field = builder.finalize();
        for row in 0..2 {
            for col in 0..3 {
                assert!(field
                    .attributes()
                    .intensity(row, col)
                    .is_some_and(f32::is_nan));
            }
        }
    }

    #[test]
    fn ranges_and_attributes_fill_in_either_order() {
        let mut builder = lidar_builder();
        builder
            .set_attributes(0, 0, LidarCell { intensity: 0.2 })
            .expect("in bounds");
        builder.set(0, 0, 1.0).expect("in bounds");
        builder.set(1, 2, 3.0).expect("in bounds");
        builder
            .set_attributes(1, 2, LidarCell { intensity: 0.9 })
            .expect("in bounds");

        let cloud = builder.finalize().to_point_cloud();
        assert_eq!(cloud.attributes().intensity(), &[0.2, 0.9]);
        assert_eq!(cloud.attributes().ring(), &[0, 1]);
    }

    #[test]
    fn a_misses_attributes_survive_finalize_but_not_the_cloud() {
        let mut builder = lidar_builder();
        builder
            .set_attributes(0, 1, LidarCell { intensity: 0.3 })
            .expect("in bounds");
        let field = builder.finalize();

        assert_eq!(field.range(0, 1), Some(NOTHING_RETURNED));
        assert_eq!(field.attributes().intensity(0, 1), Some(0.3));
        assert!(field.to_point_cloud().attributes().intensity().is_empty());
    }

    /// Guards the `Clone` impl being generic over the bundle, not just `()`:
    /// a producer clones one blank lidar builder per scan.
    #[test]
    fn a_lidar_clone_fills_its_attributes_independently_of_its_blank() {
        let blank = lidar_builder();

        let mut scan = blank.clone();
        scan.set_attributes(0, 0, LidarCell { intensity: 0.5 })
            .expect("in bounds");
        let filled = scan.finalize();
        let untouched = blank.finalize();

        assert_eq!(filled.attributes().intensity(0, 0), Some(0.5));
        assert!(untouched
            .attributes()
            .intensity(0, 0)
            .is_some_and(f32::is_nan));
    }

    #[test]
    fn debug_prints_a_summary_not_the_grid() {
        let mut builder = builder();
        builder.set(0, 0, 12345.5).expect("in bounds");
        let printed = format!("{builder:?}");

        assert!(printed.contains("shape: (2, 3)"), "{printed}");
        assert!(!printed.contains("12345.5"), "{printed}");
    }

    #[test]
    fn a_filled_timed_builder_yields_a_timed_cloud_of_its_hits() {
        let mut builder = builder()
            .with_timing(ScanTiming::PerColumn(vec![0, 100, 200]))
            .expect("fits");
        builder.set(0, 0, 1.0).expect("in bounds");
        builder.set(1, 2, 3.0).expect("in bounds");

        let cloud = builder.finalize().to_point_cloud();
        assert_eq!(cloud.len(), 2);
        assert_eq!(
            cloud.time().expect("timed").as_slice(),
            &[0, 200],
            "column-major order, each point taking its column's offset"
        );
    }
}
