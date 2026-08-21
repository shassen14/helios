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
    estimation::schema::{Quantity, StateSchema},
    frames::{
        conventions::Frame,
        quantities::{FreeVector, Point},
        transforms::{Rotation, Transform},
    },
};

use nalgebra::{
    DMatrix, DVector, Isometry3, Quaternion, Translation, Translation3, UnitQuaternion, Vector3,
};
use serde::{Deserialize, Serialize};
use std::{hash::Hash, sync::Arc};

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

/// An enum that defines every possible variable that can exist in a state vector.
/// The FrameId specifies which frame the variable is expressed in.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum StateVariable {
    // --- Cartesian Position ---
    Px(FrameId),
    Py(FrameId),
    Pz(FrameId),
    // --- Cartesian Velocity ---
    Vx(FrameId),
    Vy(FrameId),
    Vz(FrameId),
    // --- Cartesian Acceleration ---
    Ax(FrameId),
    Ay(FrameId),
    Az(FrameId),
    // --- Orientation (as a quaternion) ---
    // Represents the rotation FROM the first frame TO the second frame.
    // e.g., Qx(from: Body, to: World)
    Qx(FrameId, FrameId),
    Qy(FrameId, FrameId),
    Qz(FrameId, FrameId),
    Qw(FrameId, FrameId),
    // --- Angular Velocity ---
    Wx(FrameId),
    Wy(FrameId),
    Wz(FrameId),
    // --- Angular Acceleration --
    Alphax(FrameId),
    Alphay(FrameId),
    Alphaz(FrameId),
    // --- Magnetic Field ---
    MagX(FrameId),
    MagY(FrameId),
    MagZ(FrameId),
    // --- Bias ---
    MagBiasX(FrameId),
    MagBiasY(FrameId),
    MagBiasZ(FrameId),
    // Accelerometer / gyroscope biases carry their own names rather than
    // borrowing `Ax`/`Wx`: those denote *true* body-frame kinematic acceleration
    // and angular velocity (what a specific-force or rate model predicts), a
    // distinct quantity from the sensor error the filter estimates. Sharing one
    // name would let a kinematics reader silently pick up the bias instead.
    AccelBiasX(FrameId),
    AccelBiasY(FrameId),
    AccelBiasZ(FrameId),
    GyroBiasX(FrameId),
    GyroBiasY(FrameId),
    GyroBiasZ(FrameId),
}

#[derive(Debug, Clone, Serialize)]
pub struct RobotState {
    /// The ordered "schema" of the state vector.
    pub layout: Vec<StateVariable>,
    /// The actual numerical data vector `x`.
    pub vector: DVector<f64>,
    /// The timestamp of the last update.
    pub(crate) timestamp: f64,
}

impl RobotState {
    pub fn new(layout: Vec<StateVariable>, timestamp: f64) -> Self {
        let mut vector = DVector::zeros(layout.len());

        // Find the quaternion part of the state and initialize it to identity (0,0,0,1).
        // This is critical to prevent NaN values from an invalid zero quaternion.
        for (i, var) in layout.iter().enumerate() {
            // Using `matches!` is a clean way to check the enum variant without caring about its contents.
            if matches!(var, StateVariable::Qw(_, _)) {
                vector[i] = 1.0; // Set the 'w' component to 1.
                                 // Note: This assumes only one quaternion in the state. For more complex
                                 // states, this logic would need to be more robust.
                break;
            }
        }

        Self {
            layout,
            vector,
            timestamp,
        }
    }

    /// Returns the dimension (number of rows) of the state vector.
    pub(crate) fn dim(&self) -> usize {
        self.layout.len()
    }

    /// Finds the index of a specific `StateVariable` in the layout.
    pub(crate) fn find_idx(&self, var: &StateVariable) -> Option<usize> {
        self.layout.iter().position(|v| v == var)
    }

    /// Sets a single state variable by name. Returns `true` if the variable was found
    /// in the layout and written; `false` if it is absent (a no-op in that case).
    pub(crate) fn set_variable(&mut self, var: &StateVariable, value: f64) -> bool {
        if let Some(idx) = self.find_idx(var) {
            self.vector[idx] = value;
            true
        } else {
            false
        }
    }

    /// Extracts a 3D vector from the state vector based on a starting `StateVariable`.
    ///
    /// For example, providing `StateVariable::Px(frame_id)` will attempt to find `Px`, `Py`,
    /// and `Pz` for that frame and return them as a `Vector3`.
    ///
    /// # Arguments
    /// * `start_variable`: The `StateVariable` representing the X-component of the desired vector.
    ///
    /// # Returns
    /// * `Some(Vector3<f64>)` if all three components (X, Y, Z) are found contiguously.
    /// * `None` if any of the components are missing or not in order.
    pub fn get_vector3(&self, start_variable: &StateVariable) -> Option<Vector3<f64>> {
        // Determine the expected Y and Z variables based on the provided X variable.
        let (expected_y, expected_z) = match start_variable {
            StateVariable::Px(id) => (StateVariable::Py(id.clone()), StateVariable::Pz(id.clone())),
            StateVariable::Vx(id) => (StateVariable::Vy(id.clone()), StateVariable::Vz(id.clone())),
            StateVariable::Ax(id) => (StateVariable::Ay(id.clone()), StateVariable::Az(id.clone())),
            StateVariable::Wx(id) => (StateVariable::Wy(id.clone()), StateVariable::Wz(id.clone())),
            StateVariable::Alphax(id) => (
                StateVariable::Alphay(id.clone()),
                StateVariable::Alphaz(id.clone()),
            ),
            StateVariable::MagX(id) => (
                StateVariable::MagY(id.clone()),
                StateVariable::MagZ(id.clone()),
            ),
            _ => return None, // Not a valid start of a 3D vector
        };

        // Find the starting index.
        let start_idx = self.find_idx(start_variable)?;

        // Check if the next two elements match the expected Y and Z variables.
        // Also ensures we don't read past the end of the layout vector.
        if self.layout.get(start_idx + 1) == Some(&expected_y)
            && self.layout.get(start_idx + 2) == Some(&expected_z)
        {
            // If they match, we can safely slice the state vector.
            // `fixed_rows` provides a view into the DVector without copying.
            let vec_slice = self.vector.fixed_rows::<3>(start_idx);
            Some(vec_slice.into()) // Convert the slice into an owned Vector3
        } else {
            None // The state layout is not as expected.
        }
    }

    /// Extracts the orientation quaternion from the state vector.
    ///
    /// It searches for the `Qw` component and assumes that `Qx`, `Qy`, and `Qz`
    /// precede it in the state layout.
    ///
    /// # Returns
    /// * `Some(UnitQuaternion<f64>)` if a valid quaternion is found.
    /// * `None` if the quaternion components are not found contiguously.
    pub(crate) fn get_orientation(&self) -> Option<UnitQuaternion<f64>> {
        // We can find any of the quaternion components to start, but searching for
        // Qw is often convenient as it's the last one.
        let mut qx_idx = None;
        let mut qy_idx = None;
        let mut qz_idx = None;
        let mut qw_idx = None;

        // Find the indices of all four components.
        for (i, var) in self.layout.iter().enumerate() {
            match var {
                StateVariable::Qx(_, _) => qx_idx = Some(i),
                StateVariable::Qy(_, _) => qy_idx = Some(i),
                StateVariable::Qz(_, _) => qz_idx = Some(i),
                StateVariable::Qw(_, _) => qw_idx = Some(i),
                _ => {}
            }
        }

        // Ensure all four were found.
        let (qx_idx, qy_idx, qz_idx, qw_idx) = (qx_idx?, qy_idx?, qz_idx?, qw_idx?);

        // A robust check to ensure they are contiguous (e.g., [..., Qx, Qy, Qz, Qw, ...])
        // This makes the layout more flexible than assuming a fixed order.
        if qy_idx == qx_idx + 1 && qz_idx == qy_idx + 1 && qw_idx == qz_idx + 1 {
            // All components are contiguous, extract them.
            // Note: nalgebra's `Quaternion` constructor is `(w, i, j, k)`.
            // The `UnitQuaternion` constructor from a `Quaternion` handles normalization.
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(
                self.vector[qw_idx], // w
                self.vector[qx_idx], // x
                self.vector[qy_idx], // y
                self.vector[qz_idx], // z
            ));
            Some(quat)
        } else {
            // The quaternion components in the state layout are not contiguous.
            None
        }
    }

    /// Normalizes the quaternion components in the state vector in-place.
    /// Must be called after every predict and update step to prevent quaternion drift.
    /// No-op if no quaternion is present or if the norm is near zero (degenerate state).
    pub(crate) fn normalize_quaternion(&mut self) {
        let mut qx_idx = None;
        let mut qy_idx = None;
        let mut qz_idx = None;
        let mut qw_idx = None;
        for (i, var) in self.layout.iter().enumerate() {
            match var {
                StateVariable::Qx(_, _) => qx_idx = Some(i),
                StateVariable::Qy(_, _) => qy_idx = Some(i),
                StateVariable::Qz(_, _) => qz_idx = Some(i),
                StateVariable::Qw(_, _) => qw_idx = Some(i),
                _ => {}
            }
        }
        if let (Some(xi), Some(yi), Some(zi), Some(wi)) = (qx_idx, qy_idx, qz_idx, qw_idx) {
            let norm = (self.vector[xi].powi(2)
                + self.vector[yi].powi(2)
                + self.vector[zi].powi(2)
                + self.vector[wi].powi(2))
            .sqrt();
            if norm > 1e-9 {
                self.vector[xi] /= norm;
                self.vector[yi] /= norm;
                self.vector[zi] /= norm;
                self.vector[wi] /= norm;
            }
        }
    }

    /// Extracts the full 6-DOF pose (position and orientation) from the state vector.
    ///
    /// This method composes the results of `get_vector3` for position and
    /// `get_orientation` for the quaternion into a single `nalgebra::Isometry3`.
    ///
    /// # Returns
    /// * `Some(Isometry3<f64>)` if both the world-frame position and the orientation
    ///   quaternion are found in the state vector.
    /// * `None` if either the position or orientation components are missing.
    pub(crate) fn get_pose_isometry(&self) -> Option<Isometry3<f64>> {
        // 1. Get the position vector from the state.
        // We specifically look for position in the World frame.
        let position = self.get_vector3(&StateVariable::Px(FrameId::World))?;

        // 2. Get the orientation quaternion from the state.
        let orientation = self.get_orientation()?;

        // 3. If both were successful, combine them into an Isometry3.
        // The `?` operator above will cause the function to return `None`
        // automatically if either `get_vector3` or `get_orientation` fail.
        Some(Isometry3::from_parts(
            Translation3::from(position),
            orientation,
        ))
    }
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

    pub fn new(layout: Vec<StateVariable>, initial_covariance_val: f64, timestamp: f64) -> Self {
        let schema = Arc::new(StateSchema::degenerate(&layout));
        let dim = layout.len();
        Self {
            mean: schema.initial_value().clone(),
            covariance: DMatrix::identity(dim, dim) * initial_covariance_val,
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

    fn pose_layout() -> Vec<StateVariable> {
        vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Pz(FrameId::World),
            StateVariable::Qx(FrameId::World, FrameId::World),
            StateVariable::Qy(FrameId::World, FrameId::World),
            StateVariable::Qz(FrameId::World, FrameId::World),
            StateVariable::Qw(FrameId::World, FrameId::World),
        ]
    }

    #[test]
    fn new_seeds_identity_quaternion() {
        let s = FrameAwareState::new(pose_layout(), 1.0, 0.0);
        // The degenerate constructor seeds `Qw` to 1.0 so the state holds a valid
        // identity quaternion rather than an all-zero non-rotation.
        let qw = s
            .schema
            .storage_offset_of(&StateVariable::Qw(FrameId::World, FrameId::World))
            .unwrap();
        assert_eq!(s.mean[qw], 1.0);
    }

    #[test]
    fn set_variable_writes_reach_the_mean() {
        let mut s = FrameAwareState::new(pose_layout(), 1.0, 0.0);
        s.set_variable(&StateVariable::Px(FrameId::World), 1.0);
        s.set_variable(&StateVariable::Py(FrameId::World), 2.0);
        s.set_variable(&StateVariable::Pz(FrameId::World), 3.0);

        // `Px` heads the layout, so the three position slots are the first rows.
        assert_eq!(s.mean.fixed_rows::<3>(0), Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn set_variable_absent_is_noop() {
        let mut s = FrameAwareState::new(pose_layout(), 1.0, 0.0);
        assert!(!s.set_variable(&StateVariable::Vx(FrameId::World), 9.0));
    }

    #[test]
    fn oplus_assign_is_addition_for_degenerate_state() {
        let mut s = FrameAwareState::new(
            vec![
                StateVariable::Px(FrameId::World),
                StateVariable::Py(FrameId::World),
            ],
            1.0,
            0.0,
        );
        s.set_variable(&StateVariable::Px(FrameId::World), 1.0);
        s.set_variable(&StateVariable::Py(FrameId::World), 2.0);

        s.oplus_assign(&DVector::from_vec(vec![0.5, -0.5]));
        assert_eq!(s.mean, DVector::from_vec(vec![1.5, 1.5]));
    }

    #[test]
    fn from_schema_takes_initial_value() {
        let schema = Arc::new(StateSchema::degenerate(&pose_layout()));
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

    fn body() -> FrameId {
        FrameId::Body(FrameHandle(1))
    }

    // Isotropic 3-DOF process noise. Its value is irrelevant to a read test, but
    // a block must carry *some* noise to build — an orientation block refuses
    // `None` — so every block here is seeded with the same nonzero variance.
    fn noise() -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap())
    }

    // A composed (non-degenerate) schema: position + velocity in World, then a
    // body → World orientation. The block extractors key off `Quantity`, so the
    // state must be built through `compose`; a `degenerate` schema is a single
    // `Quantity::Raw` block and would match none of them. The orientation seeds
    // an identity quaternion `[0, 0, 0, 1]`, the flat blocks seed zeros; each
    // test overwrites the slots it reads.
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
        s.set_variable(&StateVariable::Px(FrameId::World), 1.0);
        s.set_variable(&StateVariable::Py(FrameId::World), 2.0);
        s.set_variable(&StateVariable::Pz(FrameId::World), 3.0);

        let p = s.position::<Enu>(FrameId::World).unwrap();
        assert_eq!(p.raw(), &Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn velocity_reads_the_second_block_at_its_offset() {
        // Velocity sits *after* position in storage, so a correct read proves the
        // extractor honors the block's offset rather than reading from the top.
        let mut s = composed_state();
        s.set_variable(&StateVariable::Vx(FrameId::World), 4.0);
        s.set_variable(&StateVariable::Vy(FrameId::World), 5.0);
        s.set_variable(&StateVariable::Vz(FrameId::World), 6.0);

        let v = s.velocity::<Enu>(FrameId::World).unwrap();
        assert_eq!(v.raw(), &Vector3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn orientation_reads_and_reorders_the_quaternion() {
        // A 180° rotation about Z has the exact quaternion (w, x, y, z) =
        // (0, 0, 0, 1), stored as [Qx, Qy, Qz, Qw] = [0, 0, 1, 0]. Clean scalars
        // let the reorder (w from the last slot) be checked without tolerance.
        let mut s = composed_state();
        s.set_variable(&StateVariable::Qx(body(), FrameId::World), 0.0);
        s.set_variable(&StateVariable::Qy(body(), FrameId::World), 0.0);
        s.set_variable(&StateVariable::Qz(body(), FrameId::World), 1.0);
        s.set_variable(&StateVariable::Qw(body(), FrameId::World), 0.0);

        let r = s.orientation::<Flu, Enu>(body(), FrameId::World).unwrap();
        let expected = UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0));
        assert!(quat_eq(&r.into_inner(), &expected));
    }

    #[test]
    fn pose_composes_position_and_orientation() {
        let mut s = composed_state();
        s.set_variable(&StateVariable::Px(FrameId::World), 1.0);
        s.set_variable(&StateVariable::Py(FrameId::World), 2.0);
        s.set_variable(&StateVariable::Pz(FrameId::World), 3.0);
        s.set_variable(&StateVariable::Qz(body(), FrameId::World), 1.0);
        s.set_variable(&StateVariable::Qw(body(), FrameId::World), 0.0);

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
