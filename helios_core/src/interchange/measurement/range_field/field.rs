//! The organized range grid a world sensor produces, before it is flattened to a
//! point cloud.
//!
//! [`RangeField`] keeps one range per beam in a `rows × cols` grid, with misses
//! stored instead of dropped. A miss is one of two sentinels:
//! [`NOTHING_RETURNED`] (`inf`) when the beam found nothing within range, and
//! [`NO_INFORMATION`] (`NaN`) when the cell says nothing about the world. That
//! is the information a [`PointCloud`] cannot hold: a miss is a direction with
//! no position, and the
//! grid's adjacency is lost once cells become a flat list of points. Consumers
//! that need either (free-space carving, neighbor-based filters) read the field;
//! everything else takes [`RangeField::to_point_cloud`], the one-way, lossy
//! conversion to the cloud.

use super::DirectionModel;
use crate::interchange::measurement::attribute::{
    column::AttributeColumn,
    key::{AttributeKey, Element},
    schema::AttributeSchema,
    table::AttributeTable,
};
use crate::interchange::measurement::cloud::{PointCloud, PointColumns, TimeColumn};
use crate::spatial::{conventions::Frame, quantities::Point};

use std::{fmt, marker::PhantomData, sync::Arc};

use nalgebra::{DMatrix, Matrix3xX};

/// The range of a cell whose beam travelled out to `range_max` without a
/// return. The space along its bearing is known to be empty, so a consumer may
/// carve it free. Test for it with [`f64::is_infinite`]; every blank cell holds
/// this until the producer writes a return.
pub const NOTHING_RETURNED: f64 = f64::INFINITY;

/// The range of a cell that says nothing about the world: the return was
/// closer than `range_min`, or the measurement itself was unusable. A consumer
/// must skip it, and must not carve free space along it, since something may
/// sit right in front of the sensor. Test for it with [`f64::is_nan`]; `==`
/// never matches NaN.
pub const NO_INFORMATION: f64 = f64::NAN;

/// An organized grid of ranges in frame `F`, one per beam, addressed by a
/// [`DirectionModel`].
///
/// Each cell holds the range its beam returned, in the direction model's native
/// unit, or one of the two miss sentinels, [`NOTHING_RETURNED`] and
/// [`NO_INFORMATION`]. The grid's shape is the direction model's
/// [`shape`](DirectionModel::shape), so the two cannot disagree. The field is
/// immutable once built: the crate's range-field builder is the only way to
/// make one.
///
/// Whatever else the sensor measured per cell (return intensity, for a lidar)
/// rides along as attribute grids of the same shape; a field of ranges alone
/// carries none.
pub struct RangeField<F: Frame> {
    ranges: DMatrix<f64>,
    /// One column per measured attribute, each holding a value for every cell
    /// in column-major order (see [`cell_index`]).
    attributes: AttributeTable,
    /// The attributes [`to_point_cloud`](Self::to_point_cloud) produces: the
    /// grid's own, then the direction model's derived ones.
    cloud_schema: AttributeSchema,
    direction: DirectionModel,
    range_min: f64,
    range_max: f64,
    timing: Option<ScanTiming>,
    _frame: PhantomData<F>,
}

impl<F: Frame> RangeField<F> {
    /// Assembles a field from already-checked parts, trusting `ranges` and
    /// every attribute column to match the direction model's shape, `timing` to
    /// match its axis, and `cloud_schema` to be the attributes' schema followed
    /// by the direction model's derived attributes.
    ///
    /// Crate-private because the builder is the only write path, and it is there
    /// that these are checked; this constructor deliberately does not re-check
    /// them, so the gate lives in exactly one place.
    pub(crate) fn new(
        ranges: DMatrix<f64>,
        attributes: AttributeTable,
        cloud_schema: AttributeSchema,
        direction: DirectionModel,
        range_min: f64,
        range_max: f64,
        timing: Option<ScanTiming>,
    ) -> Self {
        Self {
            ranges,
            attributes,
            cloud_schema,
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

    /// The longest range the sensor reports, in meters. A [`NOTHING_RETURNED`]
    /// cell carves free space along its bearing out to this distance; a
    /// [`NO_INFORMATION`] cell carves nothing.
    pub fn range_max(&self) -> f64 {
        self.range_max
    }

    /// When each row or column was measured, or `None` if the whole grid shares
    /// one instant (a flash capture, or a scan already corrected for motion).
    pub fn timing(&self) -> Option<&ScanTiming> {
        self.timing.as_ref()
    }

    /// The per-cell attribute grids. Unlike the cloud's columns, they keep a
    /// value for every cell, misses included, stored column by column; read
    /// one cell with [`attribute`](Self::attribute).
    pub fn attributes(&self) -> &AttributeTable {
        &self.attributes
    }

    /// The value of attribute `key` at `(row, col)`, or `None` if the field
    /// does not carry `key` or the index lies outside [`shape`](Self::shape).
    /// A cell the producer never wrote reads as the key's blank.
    pub fn attribute<T: Element>(&self, key: AttributeKey<T>, row: usize, col: usize) -> Option<T> {
        let (rows, cols) = self.shape();
        if row >= rows || col >= cols {
            return None;
        }
        self.attributes
            .get(key)?
            .get(cell_index(row, col, rows))
            .copied()
    }

    /// The attributes every cloud from [`to_point_cloud`](Self::to_point_cloud)
    /// carries, known before any flattening: the grid's own, then those the
    /// direction model derives (a lidar's `ring`).
    pub fn cloud_schema(&self) -> &AttributeSchema {
        &self.cloud_schema
    }

    /// The range stored at `(row, col)`, or `None` if the index lies outside
    /// [`shape`](Self::shape). A miss comes back as its sentinel,
    /// [`NOTHING_RETURNED`] or [`NO_INFORMATION`]; tell them apart with
    /// `is_infinite` and `is_nan`, since `NaN` never compares equal.
    pub fn range(&self, row: usize, col: usize) -> Option<f64> {
        self.ranges.get((row, col)).copied()
    }

    /// The point the beam at `(row, col)` struck, in frame `F`, or `None` for a
    /// miss of either kind or an index outside [`shape`](Self::shape).
    pub fn deproject(&self, row: usize, col: usize) -> Option<Point<F>> {
        let range = self.range(row, col)?;
        let point = self.direction.deproject(row, col, range)?;
        Some(Point::from_raw(point))
    }

    /// Flattens the grid into a [`PointCloud`], dropping misses of both kinds.
    ///
    /// Cells are visited column by column, and that order is part of the
    /// contract: for a spinning lidar a column is one azimuth, so the output
    /// is in firing order. A timed field yields a timed cloud, each point
    /// taking its row's or column's offset; an untimed field yields an untimed
    /// cloud.
    ///
    /// The cloud's attributes follow [`cloud_schema`](Self::cloud_schema): each
    /// grid's values at the kept cells, then the direction model's derived
    /// attributes. A miss's attributes stay in the field and never reach the
    /// cloud.
    ///
    /// Assembles the cloud's columns directly rather than through the cloud
    /// builder. Every kept cell pushes its time offset (when timed) before its
    /// coordinates, its keep flag, and its ring, and a cell with no offset is
    /// skipped whole, so no column can fall out of step. Going through the
    /// builder would only add checks this loop cannot fail, behind a `Result`
    /// the caller would have to handle.
    pub fn to_point_cloud(&self) -> PointCloud<F> {
        let (rows, cols) = self.shape();
        let mut coords = Vec::with_capacity(rows * cols * 3);
        let mut times = Vec::with_capacity(rows * cols);
        let mut keep = vec![false; rows * cols];
        let mut rings = Vec::with_capacity(rows * cols);

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
                if let Some(flag) = keep.get_mut(cell_index(row, col, rows)) {
                    *flag = true;
                }
                // Unreachable fallback: a direction model that derives a ring
                // rejects any row index a `u16` cannot hold.
                rings.push(u16::try_from(row).unwrap_or(u16::MAX));
            }
        }

        let geometry = PointColumns::from_arc(Arc::new(Matrix3xX::from_column_slice(&coords)));
        let time = self.timing.is_some().then(|| TimeColumn::from_vec(times));

        let mut columns = self.attributes.select(&keep).into_columns();
        columns.extend(self.derived_columns(rings));
        let attributes = AttributeTable::new(self.cloud_schema.clone(), columns);

        PointCloud::from_columns(geometry, attributes, time)
    }

    /// The values of the direction model's
    /// [`derived_attributes`](DirectionModel::derived_attributes) for the kept
    /// cells, in that list's order. `rings` holds each kept cell's row.
    ///
    /// Must produce exactly the columns that list names, since the cloud's
    /// table pairs them with it by position.
    fn derived_columns(&self, rings: Vec<u16>) -> Vec<AttributeColumn> {
        match &self.direction {
            DirectionModel::SphericalAngular(_) => vec![AttributeColumn::U16(rings.into())],
        }
    }
}

// Hand-written rather than derived: `#[derive(Clone)]` would demand `F: Clone`
// even though `F` lives only inside a `PhantomData`, and that impl is then
// invisible to generic code that knows only `F: Frame`.
impl<F: Frame> Clone for RangeField<F> {
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

/// Where cell `(row, col)` of a grid with `rows` rows sits in an attribute
/// column: column-major, the order the ranges are stored in and
/// [`RangeField::to_point_cloud`] visits cells. The builder writes by it and
/// the field reads by it, so the two cannot disagree on layout.
pub(super) fn cell_index(row: usize, col: usize, rows: usize) -> usize {
    col * rows + row
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

    use crate::interchange::measurement::attribute::canonical::{INTENSITY, RING};
    use crate::interchange::measurement::range_field::{RangeFieldBuilder, SphericalAngular};
    use crate::spatial::conventions::Flu;

    const RANGE_MIN: f64 = 0.1;
    const RANGE_MAX: f64 = 100.0;

    /// Tolerance for comparing points built from trig.
    const TOLERANCE: f64 = 1e-12;

    /// A 2-ring × 3-azimuth grid with one miss in each ring, one of each kind:
    ///
    /// ```text
    ///          col 0   col 1   col 2
    /// row 0     1.0     inf     3.0
    /// row 1     4.0     5.0     NaN
    /// ```
    ///
    /// Walked column by column, the hits are `(0,0) (1,0) (1,1) (0,2)`.
    const CELL_RANGE: [[f64; 3]; 2] = [[1.0, NOTHING_RETURNED, 3.0], [4.0, 5.0, NO_INFORMATION]];

    /// The hit cells of [`CELL_RANGE`], in column-major order.
    const HITS_COLUMN_MAJOR: [(usize, usize); 4] = [(0, 0), (1, 0), (1, 1), (0, 2)];

    /// Intensities for every cell of the grid, misses included, each distinct
    /// so a test can tell which cell a point came from:
    ///
    /// ```text
    ///          col 0   col 1   col 2
    /// row 0     0.1     0.2     0.3
    /// row 1     0.4     0.5     0.6
    /// ```
    const CELL_INTENSITY: [[f32; 3]; 2] = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]];

    fn blank_builder() -> RangeFieldBuilder<Flu> {
        let direction = DirectionModel::SphericalAngular(
            SphericalAngular::new(vec![0.0, 0.2], 0.0, 0.5, 3).expect("valid geometry"),
        );
        RangeFieldBuilder::new(direction, RANGE_MIN, RANGE_MAX).expect("valid limits")
    }

    /// Writes [`CELL_RANGE`] into `builder` and finalizes it with `timing`.
    ///
    /// The timing is attached after finalizing, unchecked, so a test can give
    /// one of the wrong length, which the builder would reject.
    fn filled(mut builder: RangeFieldBuilder<Flu>, timing: Option<ScanTiming>) -> RangeField<Flu> {
        for (row, ranges) in CELL_RANGE.iter().enumerate() {
            for (col, &range) in ranges.iter().enumerate() {
                builder.set(row, col, range).expect("in bounds");
            }
        }
        RangeField {
            timing,
            ..builder.finalize()
        }
    }

    /// The grid of [`CELL_RANGE`], ranges alone.
    fn field_with(timing: Option<ScanTiming>) -> RangeField<Flu> {
        filled(blank_builder(), timing)
    }

    /// The grid of [`CELL_RANGE`] carrying [`CELL_INTENSITY`] in an intensity
    /// grid.
    fn lidar_field_with(timing: Option<ScanTiming>) -> RangeField<Flu> {
        let schema = AttributeSchema::new([INTENSITY.descriptor()]).expect("one name");
        let mut builder = blank_builder()
            .with_attributes(schema)
            .expect("intensity has a blank");

        for (row, intensities) in CELL_INTENSITY.iter().enumerate() {
            for (col, &intensity) in intensities.iter().enumerate() {
                builder
                    .set_attribute(INTENSITY, row, col, intensity)
                    .expect("declared, in bounds");
            }
        }

        filled(builder, timing)
    }

    /// The row of each hit, walked column by column: the ring each point
    /// should carry.
    fn hit_rings() -> Vec<u16> {
        HITS_COLUMN_MAJOR
            .iter()
            .map(|&(row, _)| u16::try_from(row).expect("small test grid"))
            .collect()
    }

    fn names(schema: &AttributeSchema) -> Vec<&'static str> {
        schema.iter().map(|d| d.name()).collect()
    }

    fn assert_attribute_columns_match(cloud: &PointCloud<Flu>) {
        assert_eq!(
            cloud.attribute(INTENSITY).map(<[f32]>::len),
            Some(cloud.len())
        );
        assert_eq!(cloud.attribute(RING).map(<[u16]>::len), Some(cloud.len()));
    }

    /// The cloud's intensity column; every lidar cloud carries one.
    fn intensity_of(cloud: &PointCloud<Flu>) -> &[f32] {
        cloud
            .attribute(INTENSITY)
            .expect("a lidar cloud carries intensity")
    }

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
        assert_eq!(field.range(0, 1), Some(NOTHING_RETURNED));
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
        assert!(field.deproject(1, 2).is_none());
        assert!(field.deproject(2, 0).is_none());
    }

    #[test]
    fn the_two_miss_sentinels_survive_storage_distinguishable() {
        let field = field_with(None);
        let nothing_returned = field.range(0, 1).expect("in the grid");
        let no_information = field.range(1, 2).expect("in the grid");

        assert!(nothing_returned.is_infinite() && !nothing_returned.is_nan());
        assert!(no_information.is_nan() && !no_information.is_infinite());
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
    fn each_point_carries_its_cells_attributes_with_the_row_as_ring() {
        let cloud = lidar_field_with(None).to_point_cloud();

        let expected_intensity: Vec<f32> = HITS_COLUMN_MAJOR
            .iter()
            .map(|&(row, col)| CELL_INTENSITY[row][col])
            .collect();
        assert_eq!(intensity_of(&cloud), expected_intensity.as_slice());
        assert_eq!(cloud.attribute(RING), Some(hit_rings().as_slice()));
    }

    #[test]
    fn a_miss_keeps_its_attributes_in_the_field_but_not_the_cloud() {
        let field = lidar_field_with(None);
        let miss_intensity = CELL_INTENSITY[0][1];
        assert_eq!(field.range(0, 1), Some(NOTHING_RETURNED));

        assert_eq!(field.attribute(INTENSITY, 0, 1), Some(miss_intensity));
        assert!(!intensity_of(&field.to_point_cloud()).contains(&miss_intensity));
    }

    #[test]
    fn attribute_columns_match_the_geometry_timed_or_not() {
        assert_attribute_columns_match(&lidar_field_with(None).to_point_cloud());
        assert_attribute_columns_match(
            &lidar_field_with(Some(ScanTiming::PerColumn(vec![0, 100, 200]))).to_point_cloud(),
        );
    }

    /// The attribute-column counterpart of
    /// `a_cell_with_no_offset_is_skipped_whole`.
    #[test]
    fn a_cell_with_no_offset_pushes_no_attributes() {
        let cloud = lidar_field_with(Some(ScanTiming::PerColumn(vec![0, 100]))).to_point_cloud();

        assert_eq!(cloud.len(), 3);
        assert_attribute_columns_match(&cloud);
        assert!(!intensity_of(&cloud).contains(&CELL_INTENSITY[0][2]));
    }

    #[test]
    fn a_field_of_ranges_alone_yields_a_cloud_carrying_only_the_ring() {
        let cloud = field_with(None).to_point_cloud();

        assert_eq!(cloud.len(), HITS_COLUMN_MAJOR.len());
        assert_eq!(names(cloud.schema()), [RING.name()]);
        assert_eq!(cloud.attribute(RING), Some(hit_rings().as_slice()));
        assert_eq!(cloud.attribute(INTENSITY), None);
    }

    #[test]
    fn a_lidar_cloud_declares_intensity_then_ring() {
        let cloud = lidar_field_with(None).to_point_cloud();
        assert_eq!(names(cloud.schema()), [INTENSITY.name(), RING.name()]);
    }

    /// The field announces its cloud's schema before flattening; it must be
    /// exactly what flattening then produces.
    #[test]
    fn the_announced_cloud_schema_is_the_one_flattening_produces() {
        for field in [field_with(None), lidar_field_with(None)] {
            let cloud = field.to_point_cloud();
            assert_eq!(names(field.cloud_schema()), names(cloud.schema()));
        }
    }

    #[test]
    fn an_all_miss_field_yields_an_empty_cloud_under_the_full_schema() {
        let direction = DirectionModel::SphericalAngular(
            SphericalAngular::new(vec![0.0], 0.0, 0.5, 4).expect("valid geometry"),
        );
        let schema = AttributeSchema::new([INTENSITY.descriptor()]).expect("one name");
        let field = RangeFieldBuilder::<Flu>::new(direction, RANGE_MIN, RANGE_MAX)
            .and_then(|builder| builder.with_attributes(schema))
            .and_then(|builder| builder.with_timing(ScanTiming::PerColumn(vec![0, 1, 2, 3])))
            .expect("valid setup")
            .finalize();

        let cloud = field.to_point_cloud();
        assert!(cloud.is_empty());
        assert_eq!(cloud.time().map(TimeColumn::len), Some(0));
        assert_eq!(cloud.attribute(INTENSITY), Some([].as_slice()));
        assert_eq!(cloud.attribute(RING), Some([].as_slice()));
    }

    /// Row `u16::MAX - 1` rather than the last row: the unreachable fallback
    /// also yields `u16::MAX`, so only a row below it shows the conversion.
    #[test]
    fn a_high_ring_index_reaches_the_cloud_exact() {
        let most_rings = usize::from(u16::MAX) + 1;
        let high_row = usize::from(u16::MAX) - 1;
        let direction = DirectionModel::SphericalAngular(
            SphericalAngular::new(vec![0.0; most_rings], 0.0, 0.5, 1).expect("valid geometry"),
        );
        let mut builder =
            RangeFieldBuilder::<Flu>::new(direction, RANGE_MIN, RANGE_MAX).expect("valid limits");
        builder.set(high_row, 0, 1.0).expect("in bounds");

        let cloud = builder.finalize().to_point_cloud();
        assert_eq!(cloud.attribute(RING), Some([u16::MAX - 1].as_slice()));
    }

    #[test]
    fn attribute_reads_none_outside_the_grid_or_for_a_key_not_carried() {
        let field = lidar_field_with(None);

        assert_eq!(field.attribute(INTENSITY, 1, 2), Some(CELL_INTENSITY[1][2]));
        assert_eq!(field.attribute(INTENSITY, 2, 0), None);
        assert_eq!(field.attribute(INTENSITY, 0, 3), None);
        assert_eq!(
            field.attribute(RING, 0, 0),
            None,
            "ring is derived, not a grid"
        );
    }

    /// Attribute grids share the ranges' storage order, which is what lets
    /// flattening walk both with one index.
    #[test]
    fn cell_index_matches_the_ranges_storage_order() {
        let field = field_with(None);
        let (rows, cols) = field.shape();
        let stored = field.ranges.as_slice();

        for col in 0..cols {
            for row in 0..rows {
                let at_index = stored[cell_index(row, col, rows)];
                let at_cell = field.ranges[(row, col)];
                assert!(
                    at_index == at_cell || (at_index.is_nan() && at_cell.is_nan()),
                    "({row}, {col})"
                );
            }
        }
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
