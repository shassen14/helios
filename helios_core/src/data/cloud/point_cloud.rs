//! Struct-of-arrays point cloud: columnar geometry with parallel attribute and
//! time columns, tagged with the coordinate frame its points live in.
//!
//! [`PointCloud`] stores geometry as one `3xN` matrix rather than a `Vec` of
//! per-point structs. A whole-cloud operation is then one matrix op instead of
//! an N-length loop (see [`PointCloud::reexpress`]), and each attribute is its
//! own contiguous column that a consumer can read without a gather. The columns
//! live behind [`Arc`], so cloning a cloud — or deriving one that shares a
//! frame-invariant column — copies a pointer, not the data.

use crate::{
    data::Attributes,
    frames::{conventions::Frame, quantities::Point, transforms::Transform},
};

use core::fmt;
use std::{marker::PhantomData, sync::Arc};

use nalgebra::Matrix3xX;

/// A set of points in frame `F`, each optionally carrying attributes `A` and a
/// per-point time offset.
///
/// The three parts are stored as parallel columns of equal length: `geometry`
/// (the positions), `attributes` (intensity, ring, … — `()` for bare geometry),
/// and `time`. `A` is not "the sensor" but the *schema* of columns the cloud
/// carries, and — since the bus routes by type — its channel identity, so two
/// sensors with different attributes cannot collide on one channel.
///
/// The equal-length invariant is established once, by the builder's `finalize`,
/// and every method here preserves it.
pub struct PointCloud<F: Frame, A: Attributes = ()> {
    geometry: PointColumns<F>,
    attributes: A,
    time: Option<TimeColumn>,
}

// Hand-written rather than derived: `#[derive(Clone)]` would demand `F: Clone`
// even though `F` lives only inside a `PhantomData`, and that impl is then
// invisible to generic code that knows only `F: Frame`. Stating the true bound
// keeps the clone available there. `A: Clone` holds via the `Attributes` bound.
impl<F: Frame, A: Attributes> Clone for PointCloud<F, A> {
    fn clone(&self) -> Self {
        Self {
            geometry: self.geometry.clone(),
            attributes: self.attributes.clone(),
            time: self.time.clone(),
        }
    }
}

// Hand-written for the same reason as `Clone`, plus one of its own: a derive
// would demand `A: Debug` — which an attribute bundle like `LidarColumns` is
// not — and `F: Debug`, and would then field-dump every coordinate column. This
// prints a summary of cardinality and mode instead, so a cloud stays legible
// inside a larger debug print without constraining `A` or dumping the data.
impl<F: Frame, A: Attributes> fmt::Debug for PointCloud<F, A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PointCloud")
            .field("len", &self.len())
            .field("timed", &self.time.is_some())
            .finish()
    }
}

impl<F: Frame, A: Attributes> PointCloud<F, A> {
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
        attributes: A,
        time: Option<TimeColumn>,
    ) -> Self {
        Self {
            geometry,
            attributes,
            time,
        }
    }

    pub(crate) fn empty(attributes: A) -> Self {
        Self::from_columns(
            PointColumns::from_arc(Arc::new(Matrix3xX::zeros(0))),
            attributes,
            None,
        )
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

    /// The geometry column. A consumer that needs only positions (an endpoint
    /// mapper, ICP) takes this instead of the whole cloud, sidestepping any
    /// dependence on the attribute type `A`.
    pub fn geometry(&self) -> &PointColumns<F> {
        &self.geometry
    }

    /// The attribute columns.
    pub fn attributes(&self) -> &A {
        &self.attributes
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
    /// matrix ops rather than an N-length loop over `Transform::act`. Attributes
    /// and time are frame-invariant — a rotation does not touch intensity or a
    /// timestamp — so they are shared into the result unchanged. Only the frame
    /// tag and the geometry buffer differ, which is why this returns
    /// `PointCloud<To, A>` rather than `Self`.
    pub fn reexpress<To: Frame>(&self, t: &Transform<F, To>) -> PointCloud<To, A> {
        let iso = t.into_inner();
        let rotation = iso.rotation.to_rotation_matrix();
        let translation = iso.translation.vector;

        let mut rotated = rotation * self.geometry.raw();
        rotated
            .column_iter_mut()
            .for_each(|mut col| col += translation);

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

    use crate::data::{AttributeColumns, LidarAttrs, LidarColumns, LidarColumnsBuilder};
    use crate::frames::conventions::{Enu, Flu};
    use crate::frames::transforms::Rotation;

    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    /// A bare-geometry `Enu` cloud from `(x, y, z)` triples, no attributes or
    /// time. Built directly through the crate-private constructors so the cloud's
    /// own operations are exercised in isolation from the builder.
    fn bare(points: &[[f64; 3]]) -> PointCloud<Enu, ()> {
        // Column-major flat fill: three contiguous values become one column, and
        // an empty slice yields a valid 3x0 matrix (unlike `from_columns`, which
        // rejects zero columns).
        let flat: Vec<f64> = points.iter().flatten().copied().collect();
        PointCloud {
            geometry: PointColumns::from_arc(Arc::new(Matrix3xX::from_column_slice(&flat))),
            attributes: (),
            time: None,
        }
    }

    /// Three `Enu` points along +X carrying lidar attributes and a time column,
    /// so a test can prove the non-geometry columns travel with an operation.
    fn lidar_cloud() -> PointCloud<Enu, LidarColumns> {
        let geometry = Matrix3xX::from_columns(&[
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(3.0, 0.0, 0.0),
        ]);
        let mut builder = LidarColumnsBuilder::default();
        builder.push(LidarAttrs {
            intensity: 0.1,
            ring: 0,
        });
        builder.push(LidarAttrs {
            intensity: 0.2,
            ring: 1,
        });
        builder.push(LidarAttrs {
            intensity: 0.3,
            ring: 2,
        });
        PointCloud {
            geometry: PointColumns::from_arc(Arc::new(geometry)),
            attributes: builder.finish(),
            time: Some(TimeColumn::from_vec(vec![10, 20, 30])),
        }
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
    fn map_geometry_replaces_geometry_and_shares_the_rest() {
        let shifted = lidar_cloud().map_geometry(|g| g.map(|v| v + 100.0));

        // Geometry is the mapped matrix.
        assert_eq!(
            shifted.point(0).into_inner(),
            Vector3::new(101.0, 100.0, 100.0)
        );
        // Attributes and time pass through untouched.
        assert_eq!(shifted.attributes().intensity(), &[0.1, 0.2, 0.3]);
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

        let geometry =
            Matrix3xX::from_columns(&[Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)]);
        let mut builder = LidarColumnsBuilder::default();
        builder.push(LidarAttrs {
            intensity: 0.5,
            ring: 7,
        });
        builder.push(LidarAttrs {
            intensity: 0.6,
            ring: 8,
        });
        let source: PointCloud<Flu, LidarColumns> = PointCloud {
            geometry: PointColumns::from_arc(Arc::new(geometry)),
            attributes: builder.finish(),
            time: Some(TimeColumn::from_vec(vec![1, 2])),
        };

        let out: PointCloud<Enu, LidarColumns> = source.reexpress(&transform);

        // (1,0,0) turns to (0,1,0), then shifts +10 in X to (10,1,0).
        assert!((out.point(0).into_inner() - Vector3::new(10.0, 1.0, 0.0)).norm() < 1e-9);
        // (0,1,0) turns to (-1,0,0), then shifts +10 in X to (9,0,0).
        assert!((out.point(1).into_inner() - Vector3::new(9.0, 0.0, 0.0)).norm() < 1e-9);
        // Attributes and time are frame-invariant and ride along unchanged.
        assert_eq!(out.attributes().ring(), &[7, 8]);
        assert_eq!(out.time().unwrap().as_slice(), &[1, 2]);
    }

    #[test]
    fn select_keeps_masked_points_across_every_column() {
        let kept = lidar_cloud().select(&[true, false, true]);

        assert_eq!(kept.len(), 2);
        // Geometry: the first and third points survive.
        assert_eq!(kept.point(0).into_inner(), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(kept.point(1).into_inner(), Vector3::new(3.0, 0.0, 0.0));
        // Attributes and time filter to the same rows — proof of lockstep.
        assert_eq!(kept.attributes().intensity(), &[0.1, 0.3]);
        assert_eq!(kept.attributes().ring(), &[0, 2]);
        assert_eq!(kept.time().unwrap().as_slice(), &[10, 30]);
    }

    #[test]
    fn select_all_true_keeps_every_point() {
        assert_eq!(lidar_cloud().select(&[true, true, true]).len(), 3);
    }

    #[test]
    fn select_all_false_yields_a_valid_empty_cloud() {
        let empty = lidar_cloud().select(&[false, false, false]);

        assert!(empty.is_empty());
        assert!(empty.attributes().intensity().is_empty());
        assert_eq!(empty.time().unwrap().len(), 0);
    }

    #[test]
    fn debug_summarizes_cardinality_and_mode() {
        // Formatting an `A = ()` cloud and a `LidarColumns` cloud — which is not
        // itself `Debug` — both compile and run, proving the impl carries no
        // `A: Debug` bound. The summary reports the point count and whether a
        // time column is present, not the coordinate data.
        let untimed = format!("{:?}", bare(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]));
        assert!(untimed.contains("PointCloud"));
        assert!(untimed.contains("len: 2"));
        assert!(untimed.contains("timed: false"));

        let timed = format!("{:?}", lidar_cloud());
        assert!(timed.contains("len: 3"));
        assert!(timed.contains("timed: true")); // lidar_cloud() carries a TimeColumn
    }
}
