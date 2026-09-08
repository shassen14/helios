//! Incremental builder for [`PointCloud`]: accumulate points one at a time, then
//! freeze them into the columnar, equal-length cloud.
//!
//! A producer — a sim raycast host or a hardware driver — discovers returns one
//! at a time, in no fixed batch shape, so the cloud's parallel columns are grown
//! here as plain `Vec`s and frozen to their shared [`Arc`] form only at
//! [`finalize`](PointCloudBuilder::finalize). That final step is the single place
//! the equal-length invariant across geometry, attributes, and time is
//! established; every operation on the finished cloud then preserves it.

use crate::{
    data::{AttributeColumns, Attributes, PointCloud, PointColumns, TimeColumn},
    frames::{conventions::Frame, quantities::Point},
};

use nalgebra::Matrix3xX;
use std::{marker::PhantomData, sync::Arc};

/// Accumulates the three columns of a [`PointCloud`] as growable buffers, one
/// point per `push`.
///
/// `geometry` is flat and column-major — three `f64` per point — so `finalize`
/// hands it straight to `Matrix3xX::from_column_slice` (which, unlike
/// `from_columns`, accepts an empty buffer as a valid `3x0` matrix).
/// `attributes` is the bundle's own [`AttributeColumns`] builder, fed one row per
/// point. `time` is `Some` only when the producer records per-point offsets: the
/// `Option` *is* the timed/untimed mode, fixed at construction.
///
/// `F` fixes the frame of every point the builder accepts and of the cloud it
/// produces, even though no field stores a value of that type — hence the
/// [`PhantomData<F>`], the same device [`PointColumns`] uses to carry a frame tag
/// at no runtime cost.
pub struct PointCloudBuilder<F: Frame, A: Attributes = ()> {
    geometry: Vec<f64>,
    attributes: A::Builder,
    time: Option<Vec<i32>>,
    _reference: PhantomData<F>,
}

impl<F: Frame, A: Attributes> Default for PointCloudBuilder<F, A> {
    fn default() -> Self {
        Self::new()
    }
}

impl<F: Frame, A: Attributes> PointCloudBuilder<F, A> {
    /// An untimed builder: its points carry no per-point time offset, and the
    /// finished cloud's `time` is `None`.
    pub fn new() -> Self {
        Self {
            geometry: Vec::default(),
            attributes: A::Builder::default(),
            time: None,
            _reference: PhantomData,
        }
    }

    /// A timed builder: each point carries a per-point time offset, supplied
    /// through [`push_timed`](Self::push_timed), and the finished cloud's `time`
    /// is `Some`.
    pub fn timed() -> Self {
        Self {
            geometry: Vec::default(),
            attributes: A::Builder::default(),
            time: Some(Vec::default()),
            _reference: PhantomData,
        }
    }

    /// Appends one point and its attribute row together.
    ///
    /// Fusing the two pushes is what keeps the geometry and attribute columns
    /// advancing in lockstep, so they cannot drift apart between here and
    /// `finalize`.
    pub fn push(&mut self, point: Point<F>, row: A::Row) {
        self.geometry.extend(point.raw());
        self.attributes.push(row);
    }

    /// Appends one point, its attribute row, and its per-point time offset.
    ///
    /// On an untimed builder there is no time column to receive the offset, so it
    /// is dropped; the [`finalize`](Self::finalize) length gate — not this method
    /// — is what catches a time column left out of step with the points.
    pub fn push_timed(&mut self, point: Point<F>, row: A::Row, dt: i32) {
        self.geometry.extend(point.raw());
        self.attributes.push(row);
        if let Some(times) = self.time.as_mut() {
            times.push(dt);
        }
    }

    /// Freezes the accumulated buffers into a [`PointCloud`], or reports the
    /// column whose length disagrees with the point count.
    ///
    /// The geometry defines the point count. The attribute columns are checked
    /// through their frozen [`column_len`](Attributes::column_len): `None` (bare
    /// geometry) has no column to disagree and opts out, while `Some(n)` must
    /// equal the point count. A recorded time column must match it too. This is
    /// the one gate for the equal-length invariant, which is why it lives here
    /// and not on the finished cloud's own operations.
    pub fn finalize(self) -> Result<PointCloud<F, A>, CloudBuildError> {
        let points = self.geometry.len() / 3;
        let attributes = self.attributes.finish();

        if let Some(n) = attributes.column_len() {
            if points != n {
                return Err(CloudBuildError::AttributeLengthMismatch {
                    points,
                    attributes: n,
                });
            }
        }

        if let Some(times) = &self.time {
            if times.len() != points {
                return Err(CloudBuildError::TimeLengthMismatch {
                    points,
                    times: times.len(),
                });
            }
        }

        let geometry =
            PointColumns::from_arc(Arc::new(Matrix3xX::from_column_slice(&self.geometry)));
        let time = self.time.map(TimeColumn::from_vec);

        Ok(PointCloud::from_columns(geometry, attributes, time))
    }
}

/// Why a [`PointCloudBuilder`] could not produce a cloud: one column's length
/// disagreed with the point count, so freezing it would break the cloud's
/// equal-length invariant.
#[derive(Debug)]
pub enum CloudBuildError {
    /// The attribute columns held a different number of rows than there were
    /// points. Unreachable through the fused `push`, this guards a builder whose
    /// buffers were assembled out of step.
    AttributeLengthMismatch { points: usize, attributes: usize },
    /// A time column was recorded but its length did not match the point count —
    /// for instance, `push` was mixed with `push_timed` on a timed builder.
    TimeLengthMismatch { points: usize, times: usize },
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::data::{LidarAttrs, LidarColumns, LidarColumnsBuilder};
    use crate::frames::conventions::Enu;

    use nalgebra::Vector3;

    /// One lidar attribute row, terse enough to keep the push calls readable.
    fn attr(intensity: f32, ring: u16) -> LidarAttrs {
        LidarAttrs { intensity, ring }
    }

    #[test]
    fn new_builder_finalizes_to_an_empty_untimed_cloud() {
        let cloud = PointCloudBuilder::<Enu>::new().finalize().unwrap();

        assert!(cloud.is_empty());
        assert!(cloud.time().is_none());
    }

    #[test]
    fn bare_cloud_builds_with_points() {
        // A bare-geometry cloud (A = ()) must build with points in it: the unit
        // bundle has no columns, so the gate must opt out via `column_len()` ==
        // None rather than compare the points against a phantom zero.
        let mut builder = PointCloudBuilder::<Enu>::new();
        builder.push(Point::new(1.0, 0.0, 0.0), ());
        builder.push(Point::new(2.0, 0.0, 0.0), ());

        let cloud = builder.finalize().unwrap();

        assert_eq!(cloud.len(), 2);
        assert_eq!(cloud.point(1).into_inner(), Vector3::new(2.0, 0.0, 0.0));
    }

    #[test]
    fn push_accumulates_geometry_and_attributes_in_lockstep() {
        let mut builder = PointCloudBuilder::<Enu, LidarColumns>::new();
        builder.push(Point::new(1.0, 0.0, 0.0), attr(0.1, 0));
        builder.push(Point::new(2.0, 0.0, 0.0), attr(0.2, 1));

        let cloud = builder.finalize().unwrap();

        assert_eq!(cloud.len(), 2);
        assert_eq!(cloud.point(0).into_inner(), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(cloud.attributes().intensity(), &[0.1, 0.2]);
        assert_eq!(cloud.attributes().ring(), &[0, 1]);
        assert!(cloud.time().is_none());
    }

    #[test]
    fn timed_builder_records_each_offset() {
        let mut builder = PointCloudBuilder::<Enu, LidarColumns>::timed();
        builder.push_timed(Point::new(1.0, 0.0, 0.0), attr(0.1, 0), 10);
        builder.push_timed(Point::new(2.0, 0.0, 0.0), attr(0.2, 1), 20);

        let cloud = builder.finalize().unwrap();

        assert_eq!(cloud.time().unwrap().as_slice(), &[10, 20]);
    }

    #[test]
    fn finalize_rejects_a_time_column_shorter_than_the_points() {
        // Mixing `push` into a timed builder leaves the time column short; the
        // gate, not the push, is what catches it.
        let mut builder = PointCloudBuilder::<Enu, LidarColumns>::timed();
        builder.push(Point::new(1.0, 0.0, 0.0), attr(0.1, 0));

        match builder.finalize() {
            Err(CloudBuildError::TimeLengthMismatch { points, times }) => {
                assert_eq!(points, 1);
                assert_eq!(times, 0);
            }
            Err(other) => panic!("expected TimeLengthMismatch, got {other:?}"),
            Ok(_) => panic!("expected TimeLengthMismatch, built a cloud instead"),
        }
    }

    #[test]
    fn finalize_rejects_attribute_columns_out_of_step_with_the_points() {
        // The fused `push` keeps geometry and attributes aligned, so a desync is
        // only reachable by assembling the buffers directly — which is exactly
        // what the defensive gate exists to catch.
        let mut attributes = LidarColumnsBuilder::default();
        attributes.push(attr(0.1, 0)); // one attribute row ...

        let builder: PointCloudBuilder<Enu, LidarColumns> = PointCloudBuilder {
            geometry: vec![1.0, 0.0, 0.0, 2.0, 0.0, 0.0], // ... but two points
            attributes,
            time: None,
            _reference: PhantomData,
        };

        match builder.finalize() {
            Err(CloudBuildError::AttributeLengthMismatch { points, attributes }) => {
                assert_eq!(points, 2);
                assert_eq!(attributes, 1);
            }
            Err(other) => panic!("expected AttributeLengthMismatch, got {other:?}"),
            Ok(_) => panic!("expected AttributeLengthMismatch, built a cloud instead"),
        }
    }
}
