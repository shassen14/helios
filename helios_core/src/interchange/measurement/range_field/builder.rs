//! The one write path into a [`RangeField`].
//!
//! A grid's size is known before any data arrives, and both hosts address cells
//! by index — the sim by ray id, a hardware driver as packets land. So the
//! builder is sized from its [`DirectionModel`] up front, starts with every cell
//! a miss, and is filled in place by [`set`](RangeFieldBuilder::set) rather than
//! appended to. Every check happens at the call that could get it wrong, which
//! leaves [`finalize`](RangeFieldBuilder::finalize) nothing to reject.

use super::{DirectionModel, RangeField, ScanTiming};
use crate::spatial::conventions::Frame;

use std::marker::PhantomData;

use nalgebra::DMatrix;

/// Fills a [`RangeField`] in frame `F` cell by cell.
///
/// Holds the same parts as the field it builds. Nothing is derived: a derive
/// would require `F` itself to implement the trait (see `RangeField`), and
/// nothing needs to clone or print a half-built grid.
pub struct RangeFieldBuilder<F: Frame> {
    ranges: DMatrix<f64>,
    direction: DirectionModel,
    range_min: f64,
    range_max: f64,
    timing: Option<ScanTiming>,
    _frame: PhantomData<F>,
}

impl<F: Frame> RangeFieldBuilder<F> {
    /// Starts a grid shaped by `direction`, with every cell a miss (`inf`).
    ///
    /// Rejects range limits that are non-finite, negative, or leave no valid
    /// span (`range_min >= range_max`); consumers carve free space out to
    /// `range_max`, so it must be a real distance.
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

        Ok(Self {
            ranges: DMatrix::from_element(rows, cols, f64::INFINITY),
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
    /// The value is stored as given: the producer has already mapped a return
    /// outside the sensor's limits to `inf`. An index outside the grid is an
    /// error rather than a panic, because the index comes from the host; the
    /// caller skips that cell and keeps filling.
    pub fn set(&mut self, row: usize, col: usize, range: f64) -> Result<(), RangeFieldBuildError> {
        let shape = self.ranges.shape();
        let cell = self
            .ranges
            .get_mut((row, col))
            .ok_or(RangeFieldBuildError::OutOfBounds { row, col, shape })?;
        *cell = range;
        Ok(())
    }

    /// Freezes the grid into a [`RangeField`]. Cannot fail: shape is fixed at
    /// construction and timing was checked when it was attached.
    pub fn finalize(self) -> RangeField<F> {
        RangeField::new(
            self.ranges,
            self.direction,
            self.range_min,
            self.range_max,
            self.timing,
        )
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
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::interchange::measurement::range_field::SphericalAngular;
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

    #[test]
    fn a_fresh_builder_is_all_misses_in_the_direction_models_shape() {
        let field = builder().finalize();

        assert_eq!(field.shape(), (2, 3));
        for row in 0..2 {
            for col in 0..3 {
                assert_eq!(field.range(row, col), Some(f64::INFINITY));
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
        assert_eq!(field.range(0, 2), Some(f64::INFINITY));
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
