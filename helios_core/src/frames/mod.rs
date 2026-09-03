//! Coordinate frame types and the layout-indexed state vector.
//!
//! Provides [`FrameId`] (world/body/sensor identifiers), [`StateVariable`] (typed
//! state-vector slots), and [`FrameAwareState`] (the bundled state + covariance + layout
//! used by all filters). Index into `FrameAwareState` only via layout lookup — never
//! hardcode numeric indices.

pub mod conventions;
pub mod quantities;
pub mod transforms;

use crate::{
    data::primitives::FrameHandle,
    estimation::schema::StateSchema,
    frames::{
        conventions::Frame,
        quantities::{FreeVector, Point},
        transforms::{Rotation, Transform},
    },
    state::Quantity,
};

use nalgebra::{DMatrix, DVector, Quaternion, Translation, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::{hash::Hash, sync::Arc};

pub use crate::state::StateVariable;

/// A unique, hashable identifier for any coordinate frame in the simulation.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum FrameId {
    /// The global ENU simulation frame. The ultimate source of truth.
    #[default]
    World,
    Odom(FrameHandle),
    /// The origin of a rigid body, where dynamics are typically calculated.
    /// Identified by the agent's unique FrameHandle.
    Body(FrameHandle),
    /// The specific origin of a sensor component.
    /// Identified by the sensor's own unique FrameHandle.
    Sensor(FrameHandle),
}

/// The "smart" state object used by filters. It bundles the state estimate
/// (`mean`) with its schema, covariance, and timestamp.
#[derive(Debug, Clone, Serialize)]
pub struct FrameAwareState {
    #[serde(skip)]
    pub schema: Arc<StateSchema>,
    pub mean: DVector<f64>,
    pub covariance: DMatrix<f64>,
    pub timestamp: f64,
}

impl FrameAwareState {
    pub fn from_schema(schema: Arc<StateSchema>, timestamp: f64) -> Self {
        Self {
            mean: schema.initial_value().clone(),
            covariance: schema.initial_covariance().clone(),
            schema,
            timestamp,
        }
    }

    /// Retracts the mean in place by a tangent-space correction: `mean ⊞= delta`.
    /// The manifold-aware replacement for `mean += delta`; reduces to addition
    /// while every block is Euclidean.
    pub fn oplus_assign(&mut self, delta: &DVector<f64>) {
        self.mean = self.schema.oplus(self.mean.as_view(), delta.as_view());
    }

    pub fn storage_dim(&self) -> usize {
        self.schema.storage_dim()
    }

    pub fn tangent_dim(&self) -> usize {
        self.schema.tangent_dim()
    }

    pub fn schema(&self) -> &Arc<StateSchema> {
        &self.schema
    }

    /// The frame this state's kinematics are expressed in (its position block's
    /// frame). A handle-less consumer reads position in this frame rather than
    /// naming one, so the reader is agnostic to whether the producer is an
    /// odom-frame filter or a world-frame reference.
    pub fn reference_frame(&self) -> Option<FrameId> {
        self.schema.reference_frame()
    }

    fn find_idx(&self, var: &StateVariable) -> Option<usize> {
        self.schema.storage_offset_of(var)
    }

    // --- Typed block extractors ---
    //
    // Read one whole state block by its `Quantity` and hand back the Layer-3
    // carrier for that kind — `Point<F>` for a position, `FreeVector<F>` for a
    // rate, `Rotation<A, B>` for an orientation. Each locates its block through
    // `StateSchema::storage_offset_of_block`, so there is no name scan and no
    // contiguity check: a block owns a contiguous slice by construction, which is
    // what lets these replace the older by-name, scan-and-check readers entirely.
    //
    // The convention type parameter (`F`, or `A`/`B`) is *caller-asserted and
    // unchecked*: the schema stores only a block's `FrameId` identity, never its
    // axis convention, so nothing here verifies that the requested frame really
    // is expressed in `F`. That check arrives once the schema carries the
    // convention; until then the turbofish is a promise the caller keeps.
    //
    // Every extractor returns `None` when no block of that exact `Quantity` — the
    // kind *and* its frame identity — is in the schema. Asking a state that holds
    // only `Position(World)` for `position::<F>(some_body)` yields `None`.

    /// The position of `id`'s frame, as a `Point` in convention `F`.
    pub fn position<F: Frame>(&self, id: FrameId) -> Option<Point<F>> {
        let vec = self.block_vector3(&Quantity::Position(id))?;

        Some(Point::from_raw(vec))
    }

    /// The linear velocity of `id`'s frame, as a `FreeVector` in convention `F`.
    pub fn velocity<F: Frame>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3(&Quantity::Velocity(id))?;

        Some(FreeVector::from_raw(vec))
    }

    /// The linear acceleration of `id`'s frame, as a `FreeVector` in convention `F`.
    pub fn acceleration<F: Frame>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3(&Quantity::Acceleration(id))?;

        Some(FreeVector::from_raw(vec))
    }

    /// The angular velocity of `id`'s frame, as a `FreeVector` in convention `F`.
    pub fn angular_velocity<F: Frame>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3(&Quantity::AngularVelocity(id))?;

        Some(FreeVector::from_raw(vec))
    }

    /// The angular acceleration of `id`'s frame, as a `FreeVector` in convention `F`.
    pub fn angular_acceleration<F: Frame>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3(&Quantity::AngularAcceleration(id))?;
        Some(FreeVector::from_raw(vec))
    }

    /// The magnetometer bias for sensor `id`, as a `FreeVector` in convention `F`.
    pub fn mag_bias<F: Frame>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3(&Quantity::MagBias(id))?;
        Some(FreeVector::from_raw(vec))
    }

    /// The `from → to` orientation, as a `Rotation<A, B>`.
    ///
    /// The block stores four scalars in `[Qx, Qy, Qz, Qw]` order while
    /// `nalgebra::Quaternion::new` takes `(w, i, j, k)`, so `w` is read from the
    /// fourth slot and the vector part from the first three. The stored mean is
    /// unit by the orientation block's own retraction, but `from_quaternion`
    /// normalizes defensively before it is tagged.
    pub fn orientation<A: Frame, B: Frame>(
        &self,
        from: FrameId,
        to: FrameId,
    ) -> Option<Rotation<A, B>> {
        let off = self
            .schema
            .storage_offset_of_block(&Quantity::Orientation { from, to })?;

        let q = Quaternion::new(
            self.mean[off + 3],
            self.mean[off],
            self.mean[off + 1],
            self.mean[off + 2],
        );

        let u_q = UnitQuaternion::from_quaternion(q);

        Some(Rotation::from_unit_quaternion(u_q))
    }

    /// The full rigid pose of `body` expressed in `reference`, as a
    /// `Transform<A, B>`. Pure composition of two block reads — the `body →
    /// reference` [`orientation`](Self::orientation) and the body origin's
    /// [`position`](Self::position) *in the reference frame* (its translation) —
    /// so it needs no block of its own; there is no stored pose block. `None` if
    /// either the orientation or the reference-frame position is absent.
    pub fn pose<A: Frame, B: Frame>(
        &self,
        body: FrameId,
        reference: FrameId,
    ) -> Option<Transform<A, B>> {
        let rotation = self.orientation::<A, B>(body, reference.clone())?;
        let pos = self.position::<B>(reference)?;

        let t = Transform::from_parts(rotation, Translation::from(pos.into_inner()));

        Some(t)
    }

    /// Shared slice-and-copy for the flat (three-scalar) block extractors: the
    /// block's storage offset, then its three contiguous mean rows as a raw
    /// `Vector3`. The public extractors wrap the result in their typed carrier.
    fn block_vector3(&self, quantity: &Quantity) -> Option<Vector3<f64>> {
        let offset = self.schema.storage_offset_of_block(quantity)?;

        Some(self.mean.fixed_rows::<3>(offset).into())
    }

    /// Sets one named component. Returns `true` if the variable is in the
    /// layout and was written, `false` (a no-op) if absent.
    pub fn set_variable(&mut self, var: &StateVariable, value: f64) -> bool {
        match self.find_idx(var) {
            Some(idx) => {
                self.mean[idx] = value;
                true
            }
            None => false,
        }
    }
}

#[cfg(test)]
mod frame_aware_state_tests {
    use super::*;
    use crate::estimation::schema::SchemaBlock;
    use crate::manifold::TangentNoise;
    use crate::state::Component;

    // A composed position + orientation state in World, built from real
    // `Quantity` blocks via `compose`. The orientation block seeds the identity
    // quaternion `[0, 0, 0, 1]`; the position block seeds zeros.
    fn pose_schema() -> Arc<StateSchema> {
        Arc::new(StateSchema::compose(vec![
            SchemaBlock::new(
                Quantity::Position(FrameId::World),
                None,
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            SchemaBlock::new(
                Quantity::Orientation {
                    from: FrameId::World,
                    to: FrameId::World,
                },
                Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap()),
                DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
                DMatrix::identity(3, 3),
            ),
        ]))
    }

    fn pose_state() -> FrameAwareState {
        FrameAwareState::from_schema(pose_schema(), 0.0)
    }

    #[test]
    fn seeds_identity_quaternion() {
        let s = pose_state();
        // The orientation block seeds the identity quaternion `[0, 0, 0, 1]`, so
        // `Qw` reads back as 1.0 rather than an all-zero non-rotation.
        let qw = s
            .schema
            .storage_offset_of(&StateVariable::new(
                Quantity::Orientation {
                    from: FrameId::World,
                    to: FrameId::World,
                },
                Component::W,
            ))
            .unwrap();
        assert_eq!(s.mean[qw], 1.0);
    }

    #[test]
    fn set_variable_writes_reach_the_mean() {
        let mut s = pose_state();
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::X),
            1.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::Y),
            2.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::Z),
            3.0,
        );

        // The position block heads the layout, so its three slots are the first rows.
        assert_eq!(s.mean.fixed_rows::<3>(0), Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn set_variable_absent_is_noop() {
        let mut s = pose_state();
        assert!(!s.set_variable(
            &StateVariable::new(Quantity::Velocity(FrameId::World), Component::X),
            9.0
        ));
    }

    #[test]
    fn oplus_assign_is_addition_for_a_euclidean_block() {
        // A position block is Euclidean, so `oplus_assign` reduces to `mean += delta`.
        let mut s = FrameAwareState::from_schema(
            Arc::new(StateSchema::compose(vec![SchemaBlock::new(
                Quantity::Position(FrameId::World),
                None,
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            )])),
            0.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::X),
            1.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::Y),
            2.0,
        );

        s.oplus_assign(&DVector::from_vec(vec![0.5, -0.5, 0.0]));
        assert_eq!(s.mean, DVector::from_vec(vec![1.5, 1.5, 0.0]));
    }

    #[test]
    fn from_schema_takes_initial_value() {
        let schema = pose_schema();
        let s = FrameAwareState::from_schema(schema.clone(), 5.0);

        assert_eq!(&s.mean, schema.initial_value());
        assert_eq!(s.timestamp, 5.0);
    }
}

#[cfg(test)]
mod block_extractor_tests {
    use super::*;
    use crate::estimation::schema::SchemaBlock;
    use crate::frames::conventions::{Enu, Flu};
    use crate::manifold::TangentNoise;
    use crate::state::Component;

    fn body() -> FrameId {
        FrameId::Body(FrameHandle(1))
    }

    // Isotropic 3-DOF process noise. Its value is irrelevant to a read test, but
    // a block must carry *some* noise to build — an orientation block refuses
    // `None` — so every block here is seeded with the same nonzero variance.
    fn noise() -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap())
    }

    // A composed schema: position + velocity in World, then a body → World
    // orientation. The block extractors key off `Quantity`, so the state is
    // built through `compose`. The orientation seeds an identity quaternion
    // `[0, 0, 0, 1]`, the flat blocks seed zeros; each test overwrites the
    // slots it reads.
    fn composed_state() -> FrameAwareState {
        let schema = StateSchema::compose(vec![
            SchemaBlock::new(
                Quantity::Position(FrameId::World),
                noise(),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            SchemaBlock::new(
                Quantity::Velocity(FrameId::World),
                noise(),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            SchemaBlock::new(
                Quantity::Orientation {
                    from: body(),
                    to: FrameId::World,
                },
                noise(),
                DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
                DMatrix::identity(3, 3),
            ),
        ]);
        FrameAwareState::from_schema(Arc::new(schema), 0.0)
    }

    #[test]
    fn position_reads_its_block() {
        let mut s = composed_state();
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::X),
            1.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::Y),
            2.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::Z),
            3.0,
        );

        let p = s.position::<Enu>(FrameId::World).unwrap();
        assert_eq!(p.raw(), &Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn velocity_reads_the_second_block_at_its_offset() {
        // Velocity sits *after* position in storage, so a correct read proves the
        // extractor honors the block's offset rather than reading from the top.
        let mut s = composed_state();
        s.set_variable(
            &StateVariable::new(Quantity::Velocity(FrameId::World), Component::X),
            4.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Velocity(FrameId::World), Component::Y),
            5.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Velocity(FrameId::World), Component::Z),
            6.0,
        );

        let v = s.velocity::<Enu>(FrameId::World).unwrap();
        assert_eq!(v.raw(), &Vector3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn orientation_reads_and_reorders_the_quaternion() {
        // A 180° rotation about Z has the exact quaternion (w, x, y, z) =
        // (0, 0, 0, 1), stored as [Qx, Qy, Qz, Qw] = [0, 0, 1, 0]. Clean scalars
        // let the reorder (w from the last slot) be checked without tolerance.
        let mut s = composed_state();
        s.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body(),
                    to: FrameId::World,
                },
                Component::X,
            ),
            0.0,
        );
        s.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body(),
                    to: FrameId::World,
                },
                Component::Y,
            ),
            0.0,
        );
        s.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body(),
                    to: FrameId::World,
                },
                Component::Z,
            ),
            1.0,
        );
        s.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body(),
                    to: FrameId::World,
                },
                Component::W,
            ),
            0.0,
        );

        let r = s.orientation::<Flu, Enu>(body(), FrameId::World).unwrap();
        let expected = UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0));
        assert!(quat_eq(&r.into_inner(), &expected));
    }

    #[test]
    fn pose_composes_position_and_orientation() {
        let mut s = composed_state();
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::X),
            1.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::Y),
            2.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::World), Component::Z),
            3.0,
        );
        s.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body(),
                    to: FrameId::World,
                },
                Component::Z,
            ),
            1.0,
        );
        s.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body(),
                    to: FrameId::World,
                },
                Component::W,
            ),
            0.0,
        );

        let pose = s.pose::<Flu, Enu>(body(), FrameId::World).unwrap();
        let iso = pose.into_inner();

        // Translation is the reference-frame position …
        assert_eq!(iso.translation.vector, Vector3::new(1.0, 2.0, 3.0));
        // … and the rotation is the body → reference orientation.
        let expected = UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0));
        assert!(quat_eq(&iso.rotation, &expected));
    }

    #[test]
    fn absent_kind_is_none() {
        // No acceleration block was composed, so the read finds nothing.
        let s = composed_state();
        assert!(s.acceleration::<Enu>(FrameId::World).is_none());
    }

    #[test]
    fn wrong_frame_identity_is_none() {
        // Position exists only for World; the same kind under a different frame
        // identity is a different block, and is absent.
        let s = composed_state();
        assert!(s.position::<Flu>(body()).is_none());
    }

    // Quaternion equality up to the double cover: `q` and `-q` are the same
    // rotation, so compare the coordinate vectors allowing for a sign flip.
    fn quat_eq(a: &UnitQuaternion<f64>, b: &UnitQuaternion<f64>) -> bool {
        let (a, b) = (a.into_inner().coords, b.into_inner().coords);
        (a - b).norm() < 1e-12 || (a + b).norm() < 1e-12
    }
}
