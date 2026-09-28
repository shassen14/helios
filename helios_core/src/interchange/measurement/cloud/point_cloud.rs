//! Struct-of-arrays point cloud: columnar geometry with parallel attribute and
//! time columns, tagged with the coordinate frame its points live in.
//!
//! [`PointCloud`] stores geometry as one `3xN` matrix rather than a `Vec` of
//! per-point structs. A whole-cloud operation is then one matrix op instead of
//! an N-length loop (see [`PointCloud::reexpress`]), and each attribute is its
//! own contiguous column that a consumer can read without a gather. The columns
//! live behind [`Arc`], so cloning a cloud — or deriving one that shares a
//! frame-invariant column — copies a pointer, not the data.

use crate::interchange::measurement::attribute::key::{AttributeKey, Element, TransformMarker};
use crate::interchange::measurement::attribute::schema::AttributeSchema;
use crate::interchange::measurement::attribute::table::AttributeTable;
use crate::spatial::{conventions::Frame, quantities::Point, transforms::Transform};

use core::fmt;
use std::{marker::PhantomData, sync::Arc};

use nalgebra::Matrix3xX;

/// A set of points in frame `F`, each optionally carrying attribute values and
/// a per-point time offset.
///
/// The three parts are stored as parallel columns of equal length: `geometry`
/// (the positions), `attributes` (intensity, ring, … — an empty table for bare
/// geometry), and `time`. Attributes are data, not part of the type: every
/// cloud in frame `F` is the same type whatever its sensor reports, so a
/// consumer that needs fewer attributes than a producer offers reads the same
/// channel. What a cloud carries is its [`schema`](Self::schema).
///
/// The equal-length invariant is established once, by the builder's `finalize`,
/// and every method here preserves it.
pub struct PointCloud<F: Frame> {
    geometry: PointColumns<F>,
    attributes: AttributeTable,
    time: Option<TimeColumn>,
}

// Hand-written rather than derived: `#[derive(Clone)]` would demand `F: Clone`
// even though `F` lives only inside a `PhantomData`, and that impl is then
// invisible to generic code that knows only `F: Frame`. Stating the true bound
// keeps the clone available there.
impl<F: Frame> Clone for PointCloud<F> {
    fn clone(&self) -> Self {
        Self {
            geometry: self.geometry.clone(),
            attributes: self.attributes.clone(),
            time: self.time.clone(),
        }
    }
}

// Hand-written for the same reason as `Clone`, plus one of its own: a derive
// would demand `F: Debug` and would then field-dump every coordinate and
// attribute value. This prints a summary of cardinality and mode instead, so a
// cloud stays legible inside a larger debug print.
impl<F: Frame> fmt::Debug for PointCloud<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PointCloud")
            .field("len", &self.len())
            .field("timed", &self.time.is_some())
            .finish()
    }
}

impl<F: Frame> PointCloud<F> {
    /// Assembles a cloud from already-frozen columns, trusting them to be
    /// equal-length.
    ///
    /// The counterpart to [`PointColumns::from_arc`] and [`TimeColumn::from_vec`]:
    /// crate-private because the builder's `finalize` is the sole write path, and
    /// it is *there* — the one trust boundary a producer's loop crosses — that the
    /// equal-length invariant is checked. This constructor deliberately does not
    /// re-check it, so the gate lives in exactly one place.
    pub(crate) fn from_columns(
        geometry: PointColumns<F>,
        attributes: AttributeTable,
        time: Option<TimeColumn>,
    ) -> Self {
        Self {
            geometry,
            attributes,
            time,
        }
    }

    /// The number of points, defined by the geometry column.
    pub fn len(&self) -> usize {
        self.geometry.len()
    }

    /// Whether the cloud holds no points. An empty cloud is valid — a scan that
    /// returned nothing — and consumers no-op on it.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The geometry column, for a consumer that needs only positions (an
    /// endpoint mapper, ICP).
    pub fn geometry(&self) -> &PointColumns<F> {
        &self.geometry
    }

    /// The attributes this cloud carries: what a consumer may read through
    /// [`attribute`](Self::attribute).
    pub fn schema(&self) -> &AttributeSchema {
        self.attributes.schema()
    }

    /// The values of the attribute `key` names, one per point, or `None` when
    /// the cloud does not carry it under `key`'s whole definition (see
    /// [`AttributeTable::get`]).
    pub fn attribute<T: Element>(&self, key: AttributeKey<T>) -> Option<&[T]> {
        self.attributes.get(key)
    }

    /// The per-point time offsets, present only when the producer recorded them.
    pub fn time(&self) -> Option<&TimeColumn> {
        self.time.as_ref()
    }

    /// The `i`th point, as a frame-typed [`Point`].
    pub fn point(&self, i: usize) -> Point<F> {
        self.geometry.point(i)
    }

    /// Derives a cloud in the same frame with new geometry, sharing the
    /// attributes and time columns unchanged.
    ///
    /// `f` must preserve the column count: it maps the `3xN` geometry matrix to
    /// another `3xN` matrix (a per-point edit — noise, a scale, a rigid nudge),
    /// never adding or dropping columns. Changing the width would desync the
    /// geometry from the attribute and time columns, breaking the equal-length
    /// invariant off the one path (`finalize`) that guards it; the debug assert
    /// catches a violating `f` in development. Filtering points is
    /// [`select`](Self::select)'s job, not this one's.
    pub fn map_geometry(&self, f: impl FnOnce(&Matrix3xX<f64>) -> Matrix3xX<f64>) -> Self {
        let raw_geometry = f(self.geometry.raw());

        debug_assert_eq!(raw_geometry.ncols(), self.len());

        Self {
            geometry: PointColumns::from_arc(Arc::new(raw_geometry)),
            attributes: self.attributes.clone(),
            time: self.time.clone(),
        }
    }

    /// Re-expresses every point in a new frame `To` under one rigid transform.
    ///
    /// The struct-of-arrays payoff: it rotates the whole geometry matrix in a
    /// single matmul and translates each column, computing `R·G + t` in two
    /// matrix ops rather than an N-length loop over `Transform::act`. Time is
    /// frame-invariant, and so is every attribute whose marker is
    /// [`Scalar`](TransformMarker::Scalar) — a rotation does not touch intensity
    /// or a timestamp — so they are shared into the result unchanged. Only the
    /// frame tag and the geometry buffer differ, which is why this returns
    /// `PointCloud<To>` rather than `Self`.
    pub fn reexpress<To: Frame>(&self, t: &Transform<F, To>) -> PointCloud<To> {
        let iso = t.into_inner();
        let rotation = iso.rotation.to_rotation_matrix();
        let translation = iso.translation.vector;

        let mut rotated = rotation * self.geometry.raw();
        rotated
            .column_iter_mut()
            .for_each(|mut col| col += translation);

        // Every marker today is frame-invariant, so the table is shared
        // unchanged. No wildcard arm: a marker for a column that changes under
        // rotation (a surface normal) will not compile until handled here.
        for descriptor in self.attributes.schema().iter() {
            match descriptor.marker() {
                TransformMarker::Scalar => {}
            }
        }

        PointCloud {
            geometry: PointColumns::from_arc(Arc::new(rotated)),
            attributes: self.attributes.clone(),
            time: self.time.clone(),
        }
    }

    /// Keeps the points whose mask entry is true, rebuilding geometry,
    /// attributes, and time in lockstep so the columns stay row-aligned.
    ///
    /// `keep` is one flag per point, in point order. Computing it is the
    /// caller's job — a height band, a reflectivity threshold, a semantic label,
    /// or several such masks combined — which keeps this operation a pure
    /// structural filter, independent of any criterion. The same mask drives
    /// every column, so a point is kept or dropped across all of them together.
    pub fn select(&self, keep: &[bool]) -> Self {
        debug_assert_eq!(keep.len(), self.len());

        let indices: Vec<usize> = keep
            .iter()
            .enumerate()
            .filter_map(|(i, k)| k.then_some(i))
            .collect();

        let geometry = self.geometry.raw().select_columns(&indices);
        let attributes = self.attributes.select(keep);
        let time = self.time.as_ref().map(|t| t.select(keep));

        Self {
            geometry: PointColumns::from_arc(Arc::new(geometry)),
            attributes,
            time,
        }
    }
}

/// The columnar geometry of a cloud: `N` points as one `3xN` matrix, tagged with
/// their frame `F`. Behind an [`Arc`] so a derived cloud that keeps the same
/// geometry (or shares it as frame-invariant) copies a pointer.
pub struct PointColumns<F: Frame>(Arc<Matrix3xX<f64>>, PhantomData<F>);

// Hand-written for the same reason as [`PointCloud`]'s `Clone`: a derive would
// over-constrain `F`, hiding the impl from code that knows only `F: Frame`.
impl<F: Frame> Clone for PointColumns<F> {
    fn clone(&self) -> Self {
        Self(self.0.clone(), PhantomData)
    }
}

impl<F: Frame> PointColumns<F> {
    /// Tags a raw `3xN` matrix as columns in frame `F`. The sole write paths are
    /// the builder and the cloud's own derivations, so this stays crate-private.
    pub(crate) fn from_arc(points: Arc<Matrix3xX<f64>>) -> Self {
        Self(points, PhantomData)
    }

    /// The number of points (columns).
    pub fn len(&self) -> usize {
        self.0.ncols()
    }

    /// Whether there are no points.
    pub fn is_empty(&self) -> bool {
        self.0.ncols() == 0
    }

    /// The `i`th column as a frame-typed [`Point`].
    pub fn point(&self, i: usize) -> Point<F> {
        Point::from_raw(self.0.column(i).into())
    }

    /// The underlying matrix, for a batch consumer (mapper, transform).
    pub fn raw(&self) -> &Matrix3xX<f64> {
        &self.0
    }
}

/// Per-point acquisition times, stored as integer nanosecond offsets from the
/// enclosing reading's timestamp — small signed deltas rather than absolute
/// times, so a de-skew step can shift each point by its own capture instant.
#[derive(Clone)]
pub struct TimeColumn(Arc<[i32]>);

impl TimeColumn {
    /// Freezes accumulated offsets into a shareable column. Crate-private for the
    /// same single-write-path reason as [`PointColumns::from_arc`].
    pub(crate) fn from_vec(v: Vec<i32>) -> Self {
        Self(v.into())
    }

    /// The number of offsets (one per point).
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether there are no offsets.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The offsets in point order.
    pub fn as_slice(&self) -> &[i32] {
        &self.0
    }

    /// Keeps the offsets whose mask entry is true, in point order — the time
    /// column's half of [`PointCloud::select`], applying the same mask the
    /// geometry and attribute columns see.
    pub fn select(&self, keep: &[bool]) -> Self {
        let time: Vec<i32> = self
            .0
            .iter()
            .zip(keep)
            .filter_map(|(t, k)| k.then_some(*t))
            .collect();

        Self(time.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interchange::measurement::attribute::canonical::{INTENSITY, RING};
    use crate::interchange::measurement::attribute::column::AttributeColumn;
    use crate::spatial::conventions::{Enu, Flu};
    use crate::spatial::transforms::Rotation;

    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    /// `(x, y, z)` triples as frame-tagged geometry. Column-major flat fill:
    /// three contiguous values become one column, and an empty slice yields a
    /// valid 3x0 matrix (unlike `Matrix3xX::from_columns`, which rejects zero
    /// columns).
    fn geometry<F: Frame>(points: &[[f64; 3]]) -> PointColumns<F> {
        let flat: Vec<f64> = points.iter().flatten().copied().collect();
        PointColumns::from_arc(Arc::new(Matrix3xX::from_column_slice(&flat)))
    }

    /// Intensity and ring columns, one row per `(intensity, ring)` pair.
    fn lidar_table(rows: &[(f32, u16)]) -> AttributeTable {
        let schema = AttributeSchema::new([INTENSITY.descriptor(), RING.descriptor()])
            .expect("distinct core keys");
        let intensity: Vec<f32> = rows.iter().map(|&(i, _)| i).collect();
        let ring: Vec<u16> = rows.iter().map(|&(_, r)| r).collect();
        AttributeTable::new(
            schema,
            vec![
                AttributeColumn::F32(intensity.into()),
                AttributeColumn::U16(ring.into()),
            ],
        )
    }

    /// A bare-geometry `Enu` cloud, no attributes or time. Built through the
    /// crate-private constructor so the cloud's own operations are exercised
    /// in isolation from the builder.
    fn bare(points: &[[f64; 3]]) -> PointCloud<Enu> {
        PointCloud::from_columns(geometry(points), AttributeTable::default(), None)
    }

    /// Three `Enu` points along +X carrying intensity, ring, and time, so a
    /// test can prove the non-geometry columns travel with an operation.
    fn lidar_cloud() -> PointCloud<Enu> {
        PointCloud::from_columns(
            geometry(&[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]),
            lidar_table(&[(0.1, 0), (0.2, 1), (0.3, 2)]),
            Some(TimeColumn::from_vec(vec![10, 20, 30])),
        )
    }

    #[test]
    fn len_counts_points_and_empty_reads_empty() {
        assert_eq!(bare(&[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]).len(), 2);
        assert!(bare(&[]).is_empty());
    }

    #[test]
    fn point_returns_the_stored_position() {
        let cloud = bare(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        assert_eq!(cloud.point(1).into_inner(), Vector3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn bare_cloud_carries_an_empty_schema_and_reads_no_attribute() {
        let cloud = bare(&[[1.0, 0.0, 0.0]]);
        assert!(cloud.schema().is_empty());
        assert_eq!(cloud.attribute(INTENSITY), None);
    }

    #[test]
    fn attribute_reads_each_column_by_its_key() {
        let cloud = lidar_cloud();
        assert_eq!(cloud.schema().len(), 2);
        assert_eq!(cloud.attribute(INTENSITY), Some(&[0.1, 0.2, 0.3][..]));
        assert_eq!(cloud.attribute(RING), Some(&[0, 1, 2][..]));
    }

    #[test]
    fn map_geometry_replaces_geometry_and_shares_the_rest() {
        let shifted = lidar_cloud().map_geometry(|g| g.map(|v| v + 100.0));

        // Geometry is the mapped matrix.
        assert_eq!(
            shifted.point(0).into_inner(),
            Vector3::new(101.0, 100.0, 100.0)
        );
        // Attributes and time pass through untouched.
        assert_eq!(shifted.attribute(INTENSITY), Some(&[0.1, 0.2, 0.3][..]));
        assert_eq!(shifted.time().unwrap().as_slice(), &[10, 20, 30]);
    }

    #[test]
    fn reexpress_matches_the_rigid_transform_and_carries_attributes() {
        // A quarter turn about +Z, then a +10 shift along X, read Flu → Enu.
        let transform: Transform<Flu, Enu> = Transform::from_parts(
            Rotation::from_unit_quaternion(UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                FRAC_PI_2,
            )),
            Translation3::new(10.0, 0.0, 0.0),
        );
        let source: PointCloud<Flu> = PointCloud::from_columns(
            geometry(&[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
            lidar_table(&[(0.5, 7), (0.6, 8)]),
            Some(TimeColumn::from_vec(vec![1, 2])),
        );

        let out: PointCloud<Enu> = source.reexpress(&transform);

        // (1,0,0) turns to (0,1,0), then shifts +10 in X to (10,1,0).
        assert!((out.point(0).into_inner() - Vector3::new(10.0, 1.0, 0.0)).norm() < 1e-9);
        // (0,1,0) turns to (-1,0,0), then shifts +10 in X to (9,0,0).
        assert!((out.point(1).into_inner() - Vector3::new(9.0, 0.0, 0.0)).norm() < 1e-9);
        // Scalar attributes and time are frame-invariant and ride along.
        assert_eq!(out.schema().len(), 2);
        assert_eq!(out.attribute(RING), Some(&[7, 8][..]));
        assert_eq!(out.time().unwrap().as_slice(), &[1, 2]);
    }

    /// Scalar columns are shared into the re-expressed cloud, not copied.
    #[test]
    fn reexpress_shares_scalar_columns() {
        let source = lidar_cloud();
        let out: PointCloud<Enu> = source.reexpress(&Transform::identity());

        let before = source.attribute(INTENSITY).unwrap();
        let after = out.attribute(INTENSITY).unwrap();
        assert_eq!(before.as_ptr(), after.as_ptr());
    }

    #[test]
    fn select_keeps_masked_points_across_every_column() {
        let kept = lidar_cloud().select(&[true, false, true]);

        assert_eq!(kept.len(), 2);
        // Geometry: the first and third points survive.
        assert_eq!(kept.point(0).into_inner(), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(kept.point(1).into_inner(), Vector3::new(3.0, 0.0, 0.0));
        // Attributes and time filter to the same rows — proof of lockstep.
        assert_eq!(kept.attribute(INTENSITY), Some(&[0.1, 0.3][..]));
        assert_eq!(kept.attribute(RING), Some(&[0, 2][..]));
        assert_eq!(kept.time().unwrap().as_slice(), &[10, 30]);
    }

    #[test]
    fn select_all_true_keeps_every_point() {
        assert_eq!(lidar_cloud().select(&[true, true, true]).len(), 3);
    }

    /// An empty result still declares its attributes: a consumer checks the
    /// schema, not whether any point survived.
    #[test]
    fn select_all_false_yields_a_valid_empty_cloud() {
        let empty = lidar_cloud().select(&[false, false, false]);

        assert!(empty.is_empty());
        assert_eq!(empty.schema().len(), 2);
        assert_eq!(empty.attribute(INTENSITY), Some(&[][..]));
        assert_eq!(empty.time().unwrap().len(), 0);
    }

    #[test]
    fn debug_summarizes_cardinality_and_mode() {
        // The summary reports the point count and whether a time column is
        // present, not the coordinate or attribute data.
        let untimed = format!("{:?}", bare(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]));
        assert!(untimed.contains("PointCloud"));
        assert!(untimed.contains("len: 2"));
        assert!(untimed.contains("timed: false"));

        let timed = format!("{:?}", lidar_cloud());
        assert!(timed.contains("len: 3"));
        assert!(timed.contains("timed: true"));
    }
}
