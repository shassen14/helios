//! Typed block access on [`FrameAwareState`]: read or write one whole state
//! block by its `Quantity`, through the Layer-3 carrier for that kind —
//! `Point<F>` for a position, `FreeVector<F>` for a rate, `Rotation<A, B>` for
//! an orientation, `Transform<A, B>` for a pose.
//!
//! Each accessor locates its block through `StateSchema::storage_offset_of_block`,
//! so there is no name scan and no contiguity check: a block owns a contiguous
//! slice by construction. A setter writes the whole block at once, so a
//! quaternion is never left half-written the way four `set_variable` calls can
//! leave it. Only the mean is touched; the covariance is left as it was.
//!
//! The convention type parameter (`F`, or `A`/`B`) is checked against the
//! convention the schema declares for the block's frame, on both sides but in
//! different ways:
//!
//! - A **reader** `debug_assert`s it. A caller reading `position::<Flu>` from a
//!   frame the schema holds in ENU is a wiring bug and trips at the call site
//!   under debug. The check compiles out of release: both sides are fixed at
//!   build and startup (a `const` against a composed schema), so one that passes
//!   in test cannot later fail in release, and a per-tick release panic would
//!   break the no-panic rule.
//! - A **setter** returns the mismatch as a [`BlockWriteError`], in release too:
//!   a write lands in the estimate the whole stack reads, and a seeding site can
//!   surface the reason as a build error.
//!
//! Absent and wrong are different outcomes. The check runs only after a block
//! is found, so an *absent* block is a reader's `None` — a legitimate "this
//! state does not track that quantity" — or a setter's
//! [`BlockWriteError::Absent`], while a block that exists but is named in the
//! wrong convention asserts or errors. Asking a state that holds only
//! `Position(World)` for `position::<F>(some_body)` still yields `None`, never a
//! panic.

use crate::spatial::{
    quantities::{FreeVector, Point},
    state::Quantity,
    transforms::{Convention, ConventionOf, Rotation, Transform},
    FrameAwareState, FrameId,
};

use nalgebra::{Quaternion, Translation, UnitQuaternion, Vector3};
use std::error::Error;
use std::fmt::{self, Display};

impl FrameAwareState {
    /// The position of `id`'s frame, as a `Point` in convention `F`.
    pub fn position<F: ConventionOf>(&self, id: FrameId) -> Option<Point<F>> {
        let vec = self.block_vector3::<F>(&Quantity::Position(id))?;

        Some(Point::from_raw(vec))
    }

    /// The linear velocity of `id`'s frame, as a `FreeVector` in convention `F`.
    pub fn velocity<F: ConventionOf>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3::<F>(&Quantity::Velocity(id))?;

        Some(FreeVector::from_raw(vec))
    }

    /// The linear acceleration of `id`'s frame, as a `FreeVector` in convention `F`.
    pub fn acceleration<F: ConventionOf>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3::<F>(&Quantity::Acceleration(id))?;

        Some(FreeVector::from_raw(vec))
    }

    /// The angular velocity of `id`'s frame, as a `FreeVector` in convention `F`.
    pub fn angular_velocity<F: ConventionOf>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3::<F>(&Quantity::AngularVelocity(id))?;

        Some(FreeVector::from_raw(vec))
    }

    /// The angular acceleration of `id`'s frame, as a `FreeVector` in convention `F`.
    pub fn angular_acceleration<F: ConventionOf>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3::<F>(&Quantity::AngularAcceleration(id))?;
        Some(FreeVector::from_raw(vec))
    }

    /// The magnetometer bias for sensor `id`, as a `FreeVector` in convention `F`.
    pub fn mag_bias<F: ConventionOf>(&self, id: FrameId) -> Option<FreeVector<F>> {
        let vec = self.block_vector3::<F>(&Quantity::MagBias(id))?;
        Some(FreeVector::from_raw(vec))
    }

    /// The `from → to` orientation, as a `Rotation<A, B>`.
    ///
    /// The block stores four scalars in `[Qx, Qy, Qz, Qw]` order while
    /// `nalgebra::Quaternion::new` takes `(w, i, j, k)`, so `w` is read from the
    /// fourth slot and the vector part from the first three. The stored mean is
    /// unit by the orientation block's own retraction, but `from_quaternion`
    /// normalizes defensively before it is tagged.
    pub fn orientation<A: ConventionOf, B: ConventionOf>(
        &self,
        from: FrameId,
        to: FrameId,
    ) -> Option<Rotation<A, B>> {
        let off = self
            .schema
            .storage_offset_of_block(&Quantity::Orientation {
                from: from.clone(),
                to: to.clone(),
            })?;

        // The turbofish `A`/`B` name the conventions this caller believes the
        // `from`/`to` frames are in; the schema is the source of truth. A
        // mismatch is a read-in-the-wrong-convention wiring bug. Both endpoints
        // of a present orientation block are always in the map (compose folds
        // them in), so `None` here can only mean an inconsistent schema.
        debug_assert!(
            self.schema.convention_of(&from) == Some(A::CONVENTION)
                && self.schema.convention_of(&to) == Some(B::CONVENTION),
            "orientation {from}→{to} read as {}→{} but the schema declares {}→{}",
            A::CONVENTION,
            B::CONVENTION,
            self.schema
                .convention_of(&from)
                .map(|c| c.to_string())
                .unwrap_or_else(|| "no convention".to_string()),
            self.schema
                .convention_of(&to)
                .map(|c| c.to_string())
                .unwrap_or_else(|| "no convention".to_string()),
        );

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
    pub fn pose<A: ConventionOf, B: ConventionOf>(
        &self,
        body: FrameId,
        reference: FrameId,
    ) -> Option<Transform<A, B>> {
        let rotation = self.orientation::<A, B>(body, reference.clone())?;
        let pos = self.position::<B>(reference)?;

        let t = Transform::from_parts(rotation, Translation::from(pos.into_inner()));

        Some(t)
    }

    /// Writes the position of `id`'s frame from a `Point` in convention `F`.
    pub fn set_position<F: ConventionOf>(
        &mut self,
        id: FrameId,
        position: Point<F>,
    ) -> Result<(), BlockWriteError> {
        let offset = self.locate_vector3::<F>(Quantity::Position(id))?;
        self.write_vector3(offset, position.into_inner());
        Ok(())
    }

    /// Writes the linear velocity of `id`'s frame from a `FreeVector` in
    /// convention `F`.
    pub fn set_velocity<F: ConventionOf>(
        &mut self,
        id: FrameId,
        velocity: FreeVector<F>,
    ) -> Result<(), BlockWriteError> {
        let offset = self.locate_vector3::<F>(Quantity::Velocity(id))?;
        self.write_vector3(offset, velocity.into_inner());
        Ok(())
    }

    /// Writes the angular velocity of `id`'s frame from a `FreeVector` in
    /// convention `F`.
    pub fn set_angular_velocity<F: ConventionOf>(
        &mut self,
        id: FrameId,
        angular_velocity: FreeVector<F>,
    ) -> Result<(), BlockWriteError> {
        let offset = self.locate_vector3::<F>(Quantity::AngularVelocity(id))?;
        self.write_vector3(offset, angular_velocity.into_inner());
        Ok(())
    }

    /// Writes the `from → to` orientation from a `Rotation<A, B>`, all four
    /// stored scalars at once.
    pub fn set_orientation<A: ConventionOf, B: ConventionOf>(
        &mut self,
        from: FrameId,
        to: FrameId,
        rotation: Rotation<A, B>,
    ) -> Result<(), BlockWriteError> {
        let offset = self.locate_orientation::<A, B>(from, to)?;
        self.write_orientation(offset, rotation);
        Ok(())
    }

    /// Writes the rigid pose of `body` in `reference` from a `Transform<A, B>`:
    /// the `body → reference` orientation and the position in `reference`, the
    /// two blocks [`pose`](Self::pose) reads back.
    ///
    /// Both blocks are located and checked before either is written, so a pose
    /// that cannot be written whole leaves the mean untouched.
    pub fn set_pose<A: ConventionOf, B: ConventionOf>(
        &mut self,
        body: FrameId,
        reference: FrameId,
        pose: Transform<A, B>,
    ) -> Result<(), BlockWriteError> {
        let orientation_offset = self.locate_orientation::<A, B>(body, reference.clone())?;
        let position_offset = self.locate_vector3::<B>(Quantity::Position(reference))?;

        self.write_orientation(orientation_offset, pose.rotation());
        self.write_vector3(position_offset, pose.into_inner().translation.vector);
        Ok(())
    }

    /// Shared slice-and-copy for the flat (three-scalar) block readers: the
    /// block's storage offset, then its three contiguous mean rows as a raw
    /// `Vector3`. The public readers wrap the result in their typed carrier.
    fn block_vector3<F: ConventionOf>(&self, quantity: &Quantity) -> Option<Vector3<f64>> {
        let offset = self.schema.storage_offset_of_block(quantity)?;

        // The turbofish `F` names the convention this caller believes the block's
        // frame is in; the schema is the source of truth. A mismatch is a
        // read-in-the-wrong-convention wiring bug. This runs only on a found
        // block, whose frame compose has folded into the map, so `None` here can
        // only mean an inconsistent schema.
        if let Some(frame) = quantity.frame() {
            let declared = self.schema.convention_of(frame);
            debug_assert!(
                declared == Some(F::CONVENTION),
                "{quantity} read as {} but the schema declares its frame in {}",
                F::CONVENTION,
                declared
                    .map(|c| c.to_string())
                    .unwrap_or_else(|| "no convention".to_string()),
            );
        }

        Some(self.mean.fixed_rows::<3>(offset).into())
    }

    /// The storage offset of a three-scalar block, after checking that it
    /// exists and that `F` is the convention the schema declares for its frame.
    fn locate_vector3<F: ConventionOf>(
        &self,
        quantity: Quantity,
    ) -> Result<usize, BlockWriteError> {
        let Some(offset) = self.schema.storage_offset_of_block(&quantity) else {
            return Err(BlockWriteError::Absent(quantity));
        };
        if let Some(frame) = quantity.frame() {
            self.check_convention(frame, F::CONVENTION)?;
        }
        Ok(offset)
    }

    /// The storage offset of the `from → to` orientation block, after checking
    /// that it exists and that `A`/`B` are the conventions the schema declares
    /// for `from`/`to`.
    fn locate_orientation<A: ConventionOf, B: ConventionOf>(
        &self,
        from: FrameId,
        to: FrameId,
    ) -> Result<usize, BlockWriteError> {
        let quantity = Quantity::Orientation {
            from: from.clone(),
            to: to.clone(),
        };
        let Some(offset) = self.schema.storage_offset_of_block(&quantity) else {
            return Err(BlockWriteError::Absent(quantity));
        };
        self.check_convention(&from, A::CONVENTION)?;
        self.check_convention(&to, B::CONVENTION)?;
        Ok(offset)
    }

    fn check_convention(
        &self,
        frame: &FrameId,
        requested: Convention,
    ) -> Result<(), BlockWriteError> {
        let declared = self.schema.convention_of(frame);
        if declared == Some(requested) {
            Ok(())
        } else {
            Err(BlockWriteError::ConventionMismatch {
                frame: frame.clone(),
                requested,
                declared,
            })
        }
    }

    fn write_vector3(&mut self, offset: usize, value: Vector3<f64>) {
        self.mean.fixed_rows_mut::<3>(offset).copy_from(&value);
    }

    /// Stores the quaternion in the block's `[Qx, Qy, Qz, Qw]` order — the
    /// reverse of the reorder [`orientation`](Self::orientation) does on read.
    fn write_orientation<A: ConventionOf, B: ConventionOf>(
        &mut self,
        offset: usize,
        rotation: Rotation<A, B>,
    ) {
        let q = rotation.into_inner();
        self.mean[offset] = q.i;
        self.mean[offset + 1] = q.j;
        self.mean[offset + 2] = q.k;
        self.mean[offset + 3] = q.w;
    }
}

/// Why a typed setter on [`FrameAwareState`] could not write; the mean is left
/// untouched.
///
/// Non-exhaustive: a new reason (a non-finite value, say) must not break a
/// caller's `match`.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum BlockWriteError {
    /// The state's schema holds no block for this quantity.
    Absent(Quantity),
    /// The carrier names a convention for `frame` that the schema does not
    /// declare for it. `declared` is `None` when the schema records no
    /// convention for the frame at all.
    ConventionMismatch {
        frame: FrameId,
        requested: Convention,
        declared: Option<Convention>,
    },
}

impl Display for BlockWriteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BlockWriteError::Absent(quantity) => {
                write!(f, "the state holds no {quantity} block to write")
            }
            BlockWriteError::ConventionMismatch {
                frame,
                requested,
                declared,
            } => match declared {
                Some(declared) => write!(
                    f,
                    "{frame} written as {requested} but the schema declares it in {declared}"
                ),
                None => write!(
                    f,
                    "{frame} written as {requested} but the schema declares no convention for it"
                ),
            },
        }
    }
}

impl Error for BlockWriteError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimation::schema::{StateSchema, StateSchemaBlock};
    use crate::kernel::manifold::TangentNoise;
    use crate::prelude::AgentId;
    use crate::spatial::conventions::{Enu, Flu};
    use crate::spatial::primitives::MonotonicTime;
    use crate::spatial::state::Component;
    use crate::spatial::StateVariable;

    use nalgebra::{DMatrix, DVector, Isometry3, Translation3};
    use std::sync::Arc;

    const TOL: f64 = 1e-12;

    fn body() -> FrameId {
        FrameId::base_link(AgentId::new("test_agent"))
    }

    // A block must carry some process noise to build — an orientation block
    // refuses `None` — but its value is irrelevant to a read or a write.
    fn noise() -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap())
    }

    fn position_block() -> StateSchemaBlock {
        StateSchemaBlock::new(
            Quantity::Position(FrameId::world()),
            Convention::Enu,
            noise(),
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        )
    }

    fn orientation_block() -> StateSchemaBlock {
        StateSchemaBlock::orientation(
            body(),
            FrameId::world(),
            Convention::Flu,
            Convention::Enu,
            noise(),
            DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
            DMatrix::identity(3, 3),
        )
    }

    fn state_of(blocks: Vec<StateSchemaBlock>) -> FrameAwareState {
        FrameAwareState::from_schema(Arc::new(StateSchema::compose(blocks)), MonotonicTime(0.0))
    }

    // Position + velocity in World (ENU), then a body (FLU) → World
    // orientation. The orientation seeds an identity quaternion `[0, 0, 0, 1]`,
    // the flat blocks seed zeros; each test overwrites the slots it touches.
    fn pose_state() -> FrameAwareState {
        state_of(vec![
            position_block(),
            StateSchemaBlock::new(
                Quantity::Velocity(FrameId::world()),
                Convention::Enu,
                noise(),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            orientation_block(),
        ])
    }

    // A pose with a non-trivial attitude, so a dropped or reordered quaternion
    // component cannot read back as the same rotation.
    fn sample_pose() -> Transform<Flu, Enu> {
        Transform::from_isometry(Isometry3::from_parts(
            Translation3::new(1.0, -2.0, 0.5),
            UnitQuaternion::from_euler_angles(0.1, -0.2, 0.7),
        ))
    }

    // Seeds slots one component at a time, so the reader tests do not depend
    // on the setters they are compared against.
    fn seed(s: &mut FrameAwareState, quantity: Quantity, values: &[(Component, f64)]) {
        for (component, value) in values {
            s.set_variable(&StateVariable::new(quantity.clone(), *component), *value);
        }
    }

    fn body_to_world() -> Quantity {
        Quantity::Orientation {
            from: body(),
            to: FrameId::world(),
        }
    }

    // Quaternion equality up to the double cover: `q` and `-q` are the same
    // rotation, so compare the coordinate vectors allowing for a sign flip.
    fn quat_eq(a: &UnitQuaternion<f64>, b: &UnitQuaternion<f64>) -> bool {
        let (a, b) = (a.into_inner().coords, b.into_inner().coords);
        (a - b).norm() < TOL || (a + b).norm() < TOL
    }

    #[test]
    fn position_reads_its_block() {
        let mut s = pose_state();
        seed(
            &mut s,
            Quantity::Position(FrameId::world()),
            &[
                (Component::X, 1.0),
                (Component::Y, 2.0),
                (Component::Z, 3.0),
            ],
        );

        let p = s.position::<Enu>(FrameId::world()).unwrap();
        assert_eq!(p.raw(), &Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn velocity_reads_the_second_block_at_its_offset() {
        // Velocity sits *after* position in storage, so a correct read proves the
        // reader honors the block's offset rather than reading from the top.
        let mut s = pose_state();
        seed(
            &mut s,
            Quantity::Velocity(FrameId::world()),
            &[
                (Component::X, 4.0),
                (Component::Y, 5.0),
                (Component::Z, 6.0),
            ],
        );

        let v = s.velocity::<Enu>(FrameId::world()).unwrap();
        assert_eq!(v.raw(), &Vector3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn orientation_reads_and_reorders_the_quaternion() {
        // A 180° rotation about Z has the exact quaternion (w, x, y, z) =
        // (0, 0, 0, 1), stored as [Qx, Qy, Qz, Qw] = [0, 0, 1, 0]. Clean scalars
        // let the reorder (w from the last slot) be checked without tolerance.
        let mut s = pose_state();
        seed(
            &mut s,
            body_to_world(),
            &[
                (Component::X, 0.0),
                (Component::Y, 0.0),
                (Component::Z, 1.0),
                (Component::W, 0.0),
            ],
        );

        let r = s.orientation::<Flu, Enu>(body(), FrameId::world()).unwrap();
        let expected = UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0));
        assert!(quat_eq(&r.into_inner(), &expected));
    }

    #[test]
    fn pose_composes_position_and_orientation() {
        let mut s = pose_state();
        seed(
            &mut s,
            Quantity::Position(FrameId::world()),
            &[
                (Component::X, 1.0),
                (Component::Y, 2.0),
                (Component::Z, 3.0),
            ],
        );
        seed(
            &mut s,
            body_to_world(),
            &[(Component::Z, 1.0), (Component::W, 0.0)],
        );

        let pose = s.pose::<Flu, Enu>(body(), FrameId::world()).unwrap();
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
        let s = pose_state();
        assert!(s.acceleration::<Enu>(FrameId::world()).is_none());
    }

    #[test]
    fn wrong_frame_identity_is_none() {
        // Position exists only for World; the same kind under a different frame
        // identity is a different block, and is absent.
        let s = pose_state();
        assert!(s.position::<Flu>(body()).is_none());
    }

    #[test]
    #[should_panic(expected = "read as FLU")]
    fn flat_read_in_wrong_convention_panics() {
        // `World` is composed as ENU, so reading its (present) position block as
        // FLU is a wiring bug. The block exists, so the convention check fires
        // rather than returning `None`.
        let s = pose_state();
        let _ = s.position::<Flu>(FrameId::world());
    }

    #[test]
    #[should_panic(expected = "read as ENU→ENU")]
    fn orientation_read_in_wrong_convention_panics() {
        // The body→World block is composed as FLU→ENU. Reading the `from`
        // endpoint as ENU disagrees with the schema, so the check fires on a
        // block that is present.
        let s = pose_state();
        let _ = s.orientation::<Enu, Enu>(body(), FrameId::world());
    }

    #[test]
    fn a_pose_round_trips_through_the_reader() {
        let mut s = pose_state();
        let written = sample_pose();

        s.set_pose(body(), FrameId::world(), written).unwrap();

        let read = s.pose::<Flu, Enu>(body(), FrameId::world()).unwrap();
        let (w, r) = (written.into_inner(), read.into_inner());
        assert!((w.translation.vector - r.translation.vector).norm() < TOL);
        assert!(w.rotation.angle_to(&r.rotation) < TOL);
    }

    #[test]
    fn an_orientation_is_stored_whole_in_block_order() {
        // [Qx, Qy, Qz, Qw]: w goes last, the reverse of nalgebra's (w, i, j, k).
        let mut s = pose_state();
        let q = sample_pose().rotation().into_inner();

        s.set_orientation(body(), FrameId::world(), sample_pose().rotation())
            .unwrap();

        let off = s.schema.storage_offset_of_block(&body_to_world()).unwrap();
        assert_eq!(s.mean.rows(off, 4).as_slice(), &[q.i, q.j, q.k, q.w][..]);
    }

    #[test]
    fn a_velocity_round_trips_through_the_reader() {
        let mut s = pose_state();

        s.set_velocity(
            FrameId::world(),
            FreeVector::<Enu>::from_raw(Vector3::new(4.0, 5.0, 6.0)),
        )
        .unwrap();

        let v = s.velocity::<Enu>(FrameId::world()).unwrap();
        assert_eq!(v.into_inner(), Vector3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn writing_an_absent_block_errors_naming_the_quantity() {
        let mut s = pose_state();
        let before = s.mean.clone();

        let err = s
            .set_angular_velocity(FrameId::world(), FreeVector::<Enu>::from_raw(Vector3::z()))
            .unwrap_err();

        assert_eq!(
            err,
            BlockWriteError::Absent(Quantity::AngularVelocity(FrameId::world()))
        );
        assert!(err.to_string().contains("angular velocity"), "{err}");
        assert_eq!(s.mean, before);
    }

    #[test]
    fn writing_in_the_wrong_convention_errors() {
        // World is declared ENU; a FLU position into it is a wiring bug.
        let mut s = pose_state();
        let before = s.mean.clone();

        let err = s
            .set_position(FrameId::world(), Point::<Flu>::from_raw(Vector3::x()))
            .unwrap_err();

        assert_eq!(
            err,
            BlockWriteError::ConventionMismatch {
                frame: FrameId::world(),
                requested: Convention::Flu,
                declared: Some(Convention::Enu),
            }
        );
        assert_eq!(s.mean, before);
    }

    #[test]
    fn an_orientation_with_swapped_conventions_errors_on_its_from_frame() {
        let mut s = pose_state();

        let err = s
            .set_orientation(
                body(),
                FrameId::world(),
                Rotation::<Enu, Flu>::from_unit_quaternion(UnitQuaternion::identity()),
            )
            .unwrap_err();

        assert_eq!(
            err,
            BlockWriteError::ConventionMismatch {
                frame: body(),
                requested: Convention::Enu,
                declared: Some(Convention::Flu),
            }
        );
    }

    #[test]
    fn a_pose_that_cannot_be_written_whole_writes_nothing() {
        // The orientation block is present but there is no position to pair it
        // with: the attitude must not be written alone.
        let mut s = state_of(vec![orientation_block()]);
        let before = s.mean.clone();

        let err = s
            .set_pose(body(), FrameId::world(), sample_pose())
            .unwrap_err();

        assert_eq!(
            err,
            BlockWriteError::Absent(Quantity::Position(FrameId::world()))
        );
        assert_eq!(s.mean, before);
    }
}
