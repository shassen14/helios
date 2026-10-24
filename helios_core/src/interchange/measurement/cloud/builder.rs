//! Incremental builder for [`PointCloud`]: accumulate points one at a time,
//! attach attribute columns whole, then freeze them into the columnar,
//! equal-length cloud.
//!
//! A producer — a sim raycast host or a hardware driver — discovers returns one
//! at a time, in no fixed batch shape, so geometry and time are grown here as
//! plain `Vec`s and frozen to their shared [`Arc`] form only at
//! [`finalize`](PointCloudBuilder::finalize). Attribute columns arrive whole,
//! by key, against the schema the builder was created with. `finalize` is the
//! single place the cloud's invariants are established — every declared
//! attribute present under its declared definition, every column as long as
//! the geometry — and every operation on the finished cloud preserves them.

use super::{PointCloud, PointColumns, TimeColumn};
use crate::interchange::measurement::attribute::column::AttributeColumn;
use crate::interchange::measurement::attribute::key::{AttributeDescriptor, AttributeKey, Element};
use crate::interchange::measurement::attribute::schema::AttributeSchema;
use crate::interchange::measurement::attribute::table::AttributeTable;
use crate::spatial::{conventions::Frame, quantities::Point};

use nalgebra::Matrix3xX;
use std::{marker::PhantomData, sync::Arc};

/// Accumulates a [`PointCloud`]: geometry and time one point per `push`,
/// attribute columns whole through [`attach`](Self::attach).
///
/// `geometry` is flat and column-major — three `f64` per point — so `finalize`
/// hands it straight to `Matrix3xX::from_column_slice` (which, unlike
/// `from_columns`, accepts an empty buffer as a valid `3x0` matrix). `time` is
/// `Some` only when the producer records per-point offsets: the `Option` *is*
/// the timed/untimed mode, fixed at construction.
///
/// `schema` is what the producer promised to attach, fixed at construction;
/// `attached` is what it actually attached, in arrival order. `finalize`
/// compares the two.
///
/// `F` fixes the frame of every point the builder accepts and of the cloud it
/// produces, even though no field stores a value of that type — hence the
/// [`PhantomData<F>`], the same device [`PointColumns`] uses to carry a frame tag
/// at no runtime cost.
pub struct PointCloudBuilder<F: Frame> {
    geometry: Vec<f64>,
    schema: AttributeSchema,
    attached: Vec<(AttributeDescriptor, AttributeColumn)>,
    time: Option<Vec<i32>>,
    _reference: PhantomData<F>,
}

/// An untimed, geometry-only builder: its schema declares no attributes.
impl<F: Frame> Default for PointCloudBuilder<F> {
    fn default() -> Self {
        Self::new(AttributeSchema::default())
    }
}

impl<F: Frame> PointCloudBuilder<F> {
    /// An untimed builder for a cloud carrying the attributes `schema` declares.
    /// Its points carry no per-point time offset, and the finished cloud's
    /// `time` is `None`.
    pub fn new(schema: AttributeSchema) -> Self {
        Self {
            geometry: Vec::default(),
            schema,
            attached: Vec::default(),
            time: None,
            _reference: PhantomData,
        }
    }

    /// A timed builder for a cloud carrying the attributes `schema` declares.
    /// Each point carries a per-point time offset, supplied through
    /// [`push_timed`](Self::push_timed), and the finished cloud's `time` is
    /// `Some`.
    pub fn timed(schema: AttributeSchema) -> Self {
        Self {
            geometry: Vec::default(),
            schema,
            attached: Vec::default(),
            time: Some(Vec::default()),
            _reference: PhantomData,
        }
    }

    /// Appends one point.
    pub fn push(&mut self, point: Point<F>) {
        self.geometry.extend(point.raw());
    }

    /// Appends one point and its per-point time offset.
    ///
    /// On an untimed builder there is no time column to receive the offset, so it
    /// is dropped; the [`finalize`](Self::finalize) length gate — not this method
    /// — is what catches a time column left out of step with the points.
    pub fn push_timed(&mut self, point: Point<F>, dt: i32) {
        self.geometry.extend(point.raw());
        if let Some(times) = self.time.as_mut() {
            times.push(dt);
        }
    }

    /// Attaches the whole column `key` names, one value per point, in point
    /// order.
    ///
    /// Only records the column: whether it was declared, under the same
    /// definition, once, and at the right length is checked by
    /// [`finalize`](Self::finalize), so every rejection comes from one place.
    pub fn attach<T: Element>(&mut self, key: AttributeKey<T>, values: Vec<T>) {
        self.attached
            .push((key.descriptor(), T::wrap(values.into())));
    }

    /// Freezes the accumulated buffers into a [`PointCloud`], or reports the
    /// first way they disagree with the schema or with each other.
    ///
    /// The geometry defines the point count; a recorded time column must match
    /// it. Each attached column must be declared, under the declared definition,
    /// attached once, and one value per point. Each declared attribute must have
    /// been attached. The first failure is returned rather than a list: a
    /// failure here is a producer bug, not a configuration to list back.
    ///
    /// Columns enter the table in schema order, not attach order, since the
    /// table pairs its `i`-th column with the schema's `i`-th attribute.
    pub fn finalize(self) -> Result<PointCloud<F>, CloudBuildError> {
        let points = self.geometry.len() / 3;

        if let Some(times) = &self.time {
            if times.len() != points {
                return Err(CloudBuildError::TimeLengthMismatch {
                    points,
                    times: times.len(),
                });
            }
        }

        for (index, (descriptor, column)) in self.attached.iter().enumerate() {
            let name = descriptor.name();

            match self.schema.get(name) {
                None => return Err(CloudBuildError::UndeclaredAttribute { name }),
                Some(declared) if declared != *descriptor => {
                    return Err(CloudBuildError::RedefinedAttribute {
                        declared,
                        attached: *descriptor,
                    });
                }
                Some(_) => {}
            }

            let attached_before = self.attached[..index]
                .iter()
                .any(|(earlier, _)| earlier.name() == name);
            if attached_before {
                return Err(CloudBuildError::DuplicateAttribute { name });
            }

            if column.len() != points {
                return Err(CloudBuildError::AttributeLengthMismatch {
                    name,
                    points,
                    values: column.len(),
                });
            }
        }

        let mut columns = Vec::with_capacity(self.schema.len());
        for declared in self.schema.iter() {
            let Some((_, column)) = self.attached.iter().find(|(d, _)| *d == declared) else {
                return Err(CloudBuildError::MissingAttribute {
                    name: declared.name(),
                });
            };
            // Shares the values: a clone only bumps the column's reference count.
            columns.push(column.clone());
        }

        let geometry =
            PointColumns::from_arc(Arc::new(Matrix3xX::from_column_slice(&self.geometry)));
        let attributes = AttributeTable::new(self.schema, columns);
        let time = self.time.map(TimeColumn::from_vec);

        Ok(PointCloud::from_columns(geometry, attributes, time))
    }
}

/// Why a [`PointCloudBuilder`] could not produce a cloud: the attached columns
/// disagreed with the declared schema, or a column's length disagreed with the
/// point count.
#[derive(Debug, PartialEq, Eq)]
pub enum CloudBuildError {
    /// A column was attached under a name the schema does not declare.
    UndeclaredAttribute { name: &'static str },
    /// A column was attached under a declared name but another definition — a
    /// `u16` intensity against an `f32` declaration, say.
    RedefinedAttribute {
        declared: AttributeDescriptor,
        attached: AttributeDescriptor,
    },
    /// The same attribute was attached more than once.
    DuplicateAttribute { name: &'static str },
    /// A declared attribute was never attached.
    MissingAttribute { name: &'static str },
    /// An attached column held a different number of values than there were
    /// points.
    AttributeLengthMismatch {
        name: &'static str,
        points: usize,
        values: usize,
    },
    /// A time column was recorded but its length did not match the point count —
    /// for instance, `push` was mixed with `push_timed` on a timed builder.
    TimeLengthMismatch { points: usize, times: usize },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interchange::measurement::attribute::canonical::{INTENSITY, RING};
    use crate::interchange::measurement::attribute::key::TransformMarker;
    use crate::spatial::conventions::Enu;

    use nalgebra::Vector3;

    /// `intensity` redefined as an integer: same name, different definition.
    const INTEGER_INTENSITY: AttributeKey<u16> =
        AttributeKey::no_blank("intensity", TransformMarker::Scalar);

    /// A schema declaring intensity then ring.
    fn lidar_schema() -> AttributeSchema {
        AttributeSchema::new([INTENSITY.descriptor(), RING.descriptor()])
            .expect("distinct core keys")
    }

    /// A lidar builder holding two points and no columns yet.
    fn two_points() -> PointCloudBuilder<Enu> {
        let mut builder = PointCloudBuilder::new(lidar_schema());
        builder.push(Point::new(1.0, 0.0, 0.0));
        builder.push(Point::new(2.0, 0.0, 0.0));
        builder
    }

    #[test]
    fn default_builder_finalizes_to_an_empty_untimed_cloud() {
        let cloud = PointCloudBuilder::<Enu>::default().finalize().unwrap();

        assert!(cloud.is_empty());
        assert!(cloud.schema().is_empty());
        assert!(cloud.time().is_none());
    }

    #[test]
    fn geometry_only_cloud_builds_with_points() {
        let mut builder = PointCloudBuilder::<Enu>::default();
        builder.push(Point::new(1.0, 0.0, 0.0));
        builder.push(Point::new(2.0, 0.0, 0.0));

        let cloud = builder.finalize().unwrap();

        assert_eq!(cloud.len(), 2);
        assert_eq!(cloud.point(1).into_inner(), Vector3::new(2.0, 0.0, 0.0));
    }

    #[test]
    fn attached_columns_read_back_by_key() {
        let mut builder = two_points();
        builder.attach(INTENSITY, vec![0.1, 0.2]);
        builder.attach(RING, vec![0, 1]);

        let cloud = builder.finalize().unwrap();

        assert_eq!(cloud.len(), 2);
        assert_eq!(cloud.attribute(INTENSITY), Some(&[0.1, 0.2][..]));
        assert_eq!(cloud.attribute(RING), Some(&[0, 1][..]));
        assert!(cloud.time().is_none());
    }

    /// The table pairs columns with the schema by position, so a column
    /// attached out of order must still land under its own key.
    #[test]
    fn columns_attached_out_of_schema_order_read_back_correctly() {
        let mut builder = two_points();
        builder.attach(RING, vec![0, 1]);
        builder.attach(INTENSITY, vec![0.1, 0.2]);

        let cloud = builder.finalize().unwrap();

        assert_eq!(cloud.attribute(INTENSITY), Some(&[0.1, 0.2][..]));
        assert_eq!(cloud.attribute(RING), Some(&[0, 1][..]));
    }

    /// An all-miss scan: no points, and every declared column attached empty.
    #[test]
    fn zero_points_with_empty_columns_builds() {
        let mut builder = PointCloudBuilder::<Enu>::new(lidar_schema());
        builder.attach(INTENSITY, Vec::new());
        builder.attach(RING, Vec::new());

        let cloud = builder.finalize().unwrap();

        assert!(cloud.is_empty());
        assert_eq!(cloud.schema().len(), 2);
        assert_eq!(cloud.attribute(RING), Some(&[][..]));
    }

    #[test]
    fn timed_builder_records_each_offset() {
        let mut builder = PointCloudBuilder::<Enu>::timed(AttributeSchema::default());
        builder.push_timed(Point::new(1.0, 0.0, 0.0), 10);
        builder.push_timed(Point::new(2.0, 0.0, 0.0), 20);

        let cloud = builder.finalize().unwrap();

        assert_eq!(cloud.time().unwrap().as_slice(), &[10, 20]);
    }

    /// Mixing `push` into a timed builder leaves the time column short; the
    /// gate, not the push, is what catches it.
    #[test]
    fn finalize_rejects_a_time_column_shorter_than_the_points() {
        let mut builder = PointCloudBuilder::<Enu>::timed(AttributeSchema::default());
        builder.push(Point::new(1.0, 0.0, 0.0));

        assert_eq!(
            builder.finalize().unwrap_err(),
            CloudBuildError::TimeLengthMismatch {
                points: 1,
                times: 0
            }
        );
    }

    #[test]
    fn finalize_rejects_an_undeclared_attribute() {
        let mut builder = PointCloudBuilder::<Enu>::default();
        builder.push(Point::new(1.0, 0.0, 0.0));
        builder.attach(RING, vec![0]);

        assert_eq!(
            builder.finalize().unwrap_err(),
            CloudBuildError::UndeclaredAttribute { name: "ring" }
        );
    }

    #[test]
    fn finalize_rejects_a_redefined_attribute() {
        let mut builder = two_points();
        builder.attach(INTEGER_INTENSITY, vec![1, 2]);
        builder.attach(RING, vec![0, 1]);

        assert_eq!(
            builder.finalize().unwrap_err(),
            CloudBuildError::RedefinedAttribute {
                declared: INTENSITY.descriptor(),
                attached: INTEGER_INTENSITY.descriptor(),
            }
        );
    }

    #[test]
    fn finalize_rejects_an_attribute_attached_twice() {
        let mut builder = two_points();
        builder.attach(INTENSITY, vec![0.1, 0.2]);
        builder.attach(RING, vec![0, 1]);
        builder.attach(RING, vec![2, 3]);

        assert_eq!(
            builder.finalize().unwrap_err(),
            CloudBuildError::DuplicateAttribute { name: "ring" }
        );
    }

    #[test]
    fn finalize_rejects_a_missing_attribute() {
        let mut builder = two_points();
        builder.attach(INTENSITY, vec![0.1, 0.2]);

        assert_eq!(
            builder.finalize().unwrap_err(),
            CloudBuildError::MissingAttribute { name: "ring" }
        );
    }

    #[test]
    fn finalize_rejects_a_column_out_of_step_with_the_points() {
        let mut builder = two_points();
        builder.attach(INTENSITY, vec![0.1]);
        builder.attach(RING, vec![0, 1]);

        assert_eq!(
            builder.finalize().unwrap_err(),
            CloudBuildError::AttributeLengthMismatch {
                name: "intensity",
                points: 2,
                values: 1
            }
        );
    }
}
