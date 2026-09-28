//! The one write path into a [`RangeField`].
//!
//! A grid's size is known before any data arrives, and both hosts address cells
//! by index — the sim by ray id, a hardware driver as packets land. So the
//! builder is sized from its [`DirectionModel`] up front, starts with every cell
//! at [`NOTHING_RETURNED`], and is filled in place by
//! [`set`](RangeFieldBuilder::set) rather than appended to. Declared attribute
//! grids start at their blank values and are filled the same way, through
//! [`set_attribute`](RangeFieldBuilder::set_attribute). Every check happens at
//! the call that could get it wrong, which leaves
//! [`finalize`](RangeFieldBuilder::finalize) nothing to reject.

use super::{field::cell_index, DirectionModel, RangeField, ScanTiming, NOTHING_RETURNED};
use crate::interchange::measurement::attribute::{
    column::AttributeColumn,
    key::{AttributeKey, Element},
    schema::{AttributeSchema, SchemaError},
    table::AttributeTable,
};
use crate::spatial::conventions::Frame;

use std::{fmt, marker::PhantomData};

use nalgebra::DMatrix;

/// Fills a [`RangeField`] in frame `F` cell by cell.
///
/// Holds the same parts as the field it builds. A producer whose geometry,
/// limits, timing, and attributes are fixed can build one blank builder at
/// setup, where any rejection is a configuration error, and clone it for every
/// scan.
pub struct RangeFieldBuilder<F: Frame> {
    ranges: DMatrix<f64>,
    attributes: AttributeTable,
    cloud_schema: AttributeSchema,
    direction: DirectionModel,
    range_min: f64,
    range_max: f64,
    timing: Option<ScanTiming>,
    _frame: PhantomData<F>,
}

impl<F: Frame> RangeFieldBuilder<F> {
    /// Starts a grid shaped by `direction`, with every cell at
    /// [`NOTHING_RETURNED`] and no attribute grids. A producer then writes only
    /// the beams that came back, plus
    /// [`NO_INFORMATION`](super::NO_INFORMATION) for any beam it cannot vouch
    /// for.
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

        let attributes = AttributeTable::default();
        let cloud_schema = cloud_schema(attributes.schema(), &direction)?;
        let (rows, cols) = direction.shape();

        Ok(Self {
            ranges: DMatrix::from_element(rows, cols, NOTHING_RETURNED),
            attributes,
            cloud_schema,
            direction,
            range_min,
            range_max,
            timing: None,
            _frame: PhantomData,
        })
    }

    /// Declares the attributes the sensor measures per cell, each starting as
    /// a grid at its blank value. Leave it uncalled for a grid of ranges alone.
    /// Calling it again replaces the grids, discarding anything written.
    ///
    /// Rejects an attribute with no blank value, since its grid could not start
    /// out empty (a no-blank grid, every cell written before finalize, waits
    /// for its first producer), and one named like an attribute the direction
    /// model derives (`ring`, for a lidar), which flattening adds itself.
    pub fn with_attributes(
        mut self,
        schema: AttributeSchema,
    ) -> Result<Self, RangeFieldBuildError> {
        let (rows, cols) = self.direction.shape();

        let mut columns = Vec::with_capacity(schema.len());
        for descriptor in schema.iter() {
            let Some(column) = AttributeColumn::blank(descriptor, rows * cols) else {
                return Err(RangeFieldBuildError::AttributeWithoutBlank {
                    name: descriptor.name(),
                });
            };
            columns.push(column);
        }

        self.cloud_schema = cloud_schema(&schema, &self.direction)?;
        self.attributes = AttributeTable::new(schema, columns);
        Ok(self)
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

    /// Writes `value` into attribute `key`'s grid at `(row, col)`, replacing
    /// whatever was there.
    ///
    /// Independent of [`set`](Self::set): either may be written first, and
    /// attributes are kept whatever the cell's range, since some sensors report
    /// them for misses too (ambient light, for one). An index outside the grid
    /// is an error rather than a panic, for the same reason as `set`, and so is
    /// a key [`with_attributes`](Self::with_attributes) did not declare under
    /// the same definition.
    ///
    /// The first write to a grid in a builder cloned from a blank one copies
    /// that grid, so the blank stays blank for the next scan.
    pub fn set_attribute<T: Element>(
        &mut self,
        key: AttributeKey<T>,
        row: usize,
        col: usize,
        value: T,
    ) -> Result<(), RangeFieldBuildError> {
        self.check_cell(row, col)?;
        let rows = self.ranges.nrows();

        let Some(grid) = self.attributes.get_mut(key) else {
            return Err(RangeFieldBuildError::UndeclaredAttribute { name: key.name() });
        };
        if let Some(slot) = grid.get_mut(cell_index(row, col, rows)) {
            *slot = value;
        }
        Ok(())
    }

    /// Freezes the grid into a [`RangeField`]. Cannot fail: shape is fixed at
    /// construction, and timing and attributes were checked when attached.
    pub fn finalize(self) -> RangeField<F> {
        RangeField::new(
            self.ranges,
            self.attributes,
            self.cloud_schema,
            self.direction,
            self.range_min,
            self.range_max,
            self.timing,
        )
    }

    /// Whether `(row, col)` lies inside the grid. The ranges carry the shape
    /// for the whole builder; every attribute grid has the same shape.
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
impl<F: Frame> Clone for RangeFieldBuilder<F> {
    fn clone(&self) -> Self {
        Self {
            ranges: self.ranges.clone(),
            attributes: self.attributes.clone(),
            cloud_schema: self.cloud_schema.clone(),
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
impl<F: Frame> fmt::Debug for RangeFieldBuilder<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RangeFieldBuilder")
            .field("shape", &self.ranges.shape())
            .field("range_min", &self.range_min)
            .field("range_max", &self.range_max)
            .field("timed", &self.timing.is_some())
            .finish()
    }
}

/// The schema of the clouds a field flattens to: the grid's attributes, then
/// the ones `direction` derives. Computed once, when the grids are declared,
/// so flattening has no schema left to check.
fn cloud_schema(
    grid: &AttributeSchema,
    direction: &DirectionModel,
) -> Result<AttributeSchema, RangeFieldBuildError> {
    let derived = direction.derived_attributes().iter().copied();

    AttributeSchema::new(grid.iter().chain(derived)).map_err(|error| match error {
        SchemaError::DuplicateNames { names } => RangeFieldBuildError::AttributeNameTaken { names },
    })
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
    /// A declared attribute has no blank value, so its grid cannot start out
    /// empty.
    AttributeWithoutBlank { name: &'static str },
    /// These declared attributes share a name with one the direction model
    /// derives when flattening.
    AttributeNameTaken { names: Vec<&'static str> },
    /// A write named an attribute the builder did not declare, or declared
    /// under another definition.
    UndeclaredAttribute { name: &'static str },
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::interchange::measurement::attribute::canonical::{INTENSITY, RING};
    use crate::interchange::measurement::attribute::key::TransformMarker;
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

    /// A builder declaring one intensity grid, as a lidar producer would.
    fn lidar_builder() -> RangeFieldBuilder<Flu> {
        let schema = AttributeSchema::new([INTENSITY.descriptor()]).expect("one name");
        builder()
            .with_attributes(schema)
            .expect("intensity has a blank")
    }

    /// Whether every cell of `field`'s intensity grid is still blank.
    fn intensity_all_blank(field: &RangeField<Flu>) -> bool {
        (0..2).all(|row| {
            (0..3).all(|col| {
                field
                    .attribute(INTENSITY, row, col)
                    .is_some_and(f32::is_nan)
            })
        })
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
        assert!(intensity_all_blank(&lidar_builder().finalize()));
    }

    #[test]
    fn with_attributes_rejects_an_attribute_with_no_blank() {
        let schema = AttributeSchema::new([INTENSITY.descriptor(), RING.descriptor()])
            .expect("distinct names");

        assert_eq!(
            builder().with_attributes(schema).err(),
            Some(RangeFieldBuildError::AttributeWithoutBlank { name: "ring" })
        );
    }

    #[test]
    fn with_attributes_rejects_a_name_the_direction_model_derives() {
        const MEASURED_RING: AttributeKey<f32> =
            AttributeKey::nan_blank("ring", TransformMarker::Scalar);
        let schema = AttributeSchema::new([MEASURED_RING.descriptor()]).expect("one name");

        assert_eq!(
            builder().with_attributes(schema).err(),
            Some(RangeFieldBuildError::AttributeNameTaken {
                names: vec!["ring"]
            })
        );
    }

    #[test]
    fn the_cloud_schema_is_the_grids_then_the_derived_ring() {
        let names = |field: RangeField<Flu>| -> Vec<&str> {
            field.cloud_schema().iter().map(|d| d.name()).collect()
        };

        assert_eq!(names(builder().finalize()), [RING.name()]);
        assert_eq!(
            names(lidar_builder().finalize()),
            [INTENSITY.name(), RING.name()]
        );
    }

    #[test]
    fn set_attribute_writes_the_cell_and_leaves_the_rest_blank() {
        let mut builder = lidar_builder();
        builder
            .set_attribute(INTENSITY, 1, 2, 0.7)
            .expect("declared, in bounds");
        let field = builder.finalize();

        assert_eq!(field.attribute(INTENSITY, 1, 2), Some(0.7));
        assert!(field.attribute(INTENSITY, 0, 2).is_some_and(f32::is_nan));
        assert!(field.attribute(INTENSITY, 1, 1).is_some_and(f32::is_nan));
    }

    #[test]
    fn set_attribute_out_of_bounds_is_an_error_and_leaves_the_grid_untouched() {
        let mut builder = lidar_builder();

        assert_eq!(
            builder.set_attribute(INTENSITY, 2, 0, 0.7),
            Err(RangeFieldBuildError::OutOfBounds {
                row: 2,
                col: 0,
                shape: (2, 3)
            })
        );
        assert_eq!(
            builder.set_attribute(INTENSITY, 0, 3, 0.7),
            Err(RangeFieldBuildError::OutOfBounds {
                row: 0,
                col: 3,
                shape: (2, 3)
            })
        );
        assert!(intensity_all_blank(&builder.finalize()));
    }

    #[test]
    fn set_attribute_rejects_an_undeclared_or_redefined_key() {
        const UNBLANKED_INTENSITY: AttributeKey<f32> =
            AttributeKey::no_blank("intensity", TransformMarker::Scalar);

        assert_eq!(
            builder().set_attribute(INTENSITY, 0, 0, 0.7),
            Err(RangeFieldBuildError::UndeclaredAttribute { name: "intensity" })
        );
        assert_eq!(
            lidar_builder().set_attribute(UNBLANKED_INTENSITY, 0, 0, 0.7),
            Err(RangeFieldBuildError::UndeclaredAttribute { name: "intensity" })
        );
    }

    #[test]
    fn ranges_and_attributes_fill_in_either_order() {
        let mut builder = lidar_builder();
        builder
            .set_attribute(INTENSITY, 0, 0, 0.2)
            .expect("declared, in bounds");
        builder.set(0, 0, 1.0).expect("in bounds");
        builder.set(1, 2, 3.0).expect("in bounds");
        builder
            .set_attribute(INTENSITY, 1, 2, 0.9)
            .expect("declared, in bounds");

        let cloud = builder.finalize().to_point_cloud();
        assert_eq!(cloud.attribute(INTENSITY), Some([0.2, 0.9].as_slice()));
        assert_eq!(cloud.attribute(RING), Some([0, 1].as_slice()));
    }

    #[test]
    fn a_misses_attributes_survive_finalize_but_not_the_cloud() {
        let mut builder = lidar_builder();
        builder
            .set_attribute(INTENSITY, 0, 1, 0.3)
            .expect("declared, in bounds");
        let field = builder.finalize();

        assert_eq!(field.range(0, 1), Some(NOTHING_RETURNED));
        assert_eq!(field.attribute(INTENSITY, 0, 1), Some(0.3));
        assert_eq!(
            field.to_point_cloud().attribute(INTENSITY),
            Some([].as_slice())
        );
    }

    /// A producer clones one blank builder per scan, and the grids are shared
    /// until written: the first write must copy, never reach the blank.
    #[test]
    fn a_lidar_clone_fills_its_attributes_independently_of_its_blank() {
        let blank = lidar_builder();

        let mut scan = blank.clone();
        scan.set_attribute(INTENSITY, 0, 0, 0.5)
            .expect("declared, in bounds");
        let filled = scan.finalize();
        let untouched = blank.finalize();

        assert_eq!(filled.attribute(INTENSITY, 0, 0), Some(0.5));
        assert!(intensity_all_blank(&untouched));
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
