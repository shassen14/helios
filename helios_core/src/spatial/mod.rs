//! Geometry and frame vocabulary: where things are and how state is laid out.
//!
//! Provides [`FrameId`] (world/body/sensor identifiers), [`StateVariable`] (typed
//! state-vector slots), and [`FrameAwareState`] (the bundled state + covariance + layout
//! used by all filters). Index into `FrameAwareState` only via layout lookup — never
//! hardcode numeric indices. Also holds the frame `primitives` (`AgentId`, monotonic
//! time), the `tf` provider port, the `state` layout blocks, and the compile-time
//! frame typing under `conventions`.

pub mod conventions;
pub mod id;
pub mod primitives;
pub mod quantities;
pub mod state;
pub mod tf;
pub mod transforms;

mod block_access;

use crate::{estimation::schema::StateSchema, spatial::primitives::MonotonicTime};

use nalgebra::{DMatrix, DVector};
use serde::Serialize;
use std::sync::Arc;

pub use crate::spatial::state::StateVariable;
pub use block_access::BlockWriteError;
pub use id::FrameId;

/// The "smart" state object used by filters. It bundles the state estimate
/// (`mean`) with its schema, covariance, and valid-at time.
#[derive(Debug, Clone, Serialize)]
pub struct FrameAwareState {
    #[serde(skip)]
    pub schema: Arc<StateSchema>,
    pub mean: DVector<f64>,
    pub covariance: DMatrix<f64>,
    /// The instant this mean and covariance hold at — not when they were
    /// computed or published. A filter advances it by the step it predicts
    /// across and never reads a clock; whoever builds the state seeds it.
    pub timestamp: MonotonicTime,
}

impl FrameAwareState {
    pub fn from_schema(schema: Arc<StateSchema>, timestamp: MonotonicTime) -> Self {
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
    use crate::estimation::schema::StateSchemaBlock;
    use crate::kernel::manifold::TangentNoise;
    use crate::spatial::state::{Component, Quantity};
    use crate::spatial::transforms::Convention;

    use nalgebra::Vector3;

    // A composed position + orientation state in World, built from real
    // `Quantity` blocks via `compose`. The orientation block seeds the identity
    // quaternion `[0, 0, 0, 1]`; the position block seeds zeros.
    fn pose_schema() -> Arc<StateSchema> {
        Arc::new(StateSchema::compose(vec![
            StateSchemaBlock::new(
                Quantity::Position(FrameId::world()),
                Convention::Enu,
                None,
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            StateSchemaBlock::orientation(
                FrameId::world(),
                FrameId::world(),
                Convention::Enu,
                Convention::Enu,
                Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap()),
                DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
                DMatrix::identity(3, 3),
            ),
        ]))
    }

    fn pose_state() -> FrameAwareState {
        FrameAwareState::from_schema(pose_schema(), MonotonicTime(0.0))
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
                    from: FrameId::world(),
                    to: FrameId::world(),
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
            &StateVariable::new(Quantity::Position(FrameId::world()), Component::X),
            1.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::world()), Component::Y),
            2.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::world()), Component::Z),
            3.0,
        );

        // The position block heads the layout, so its three slots are the first rows.
        assert_eq!(s.mean.fixed_rows::<3>(0), Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn set_variable_absent_is_noop() {
        let mut s = pose_state();
        assert!(!s.set_variable(
            &StateVariable::new(Quantity::Velocity(FrameId::world()), Component::X),
            9.0
        ));
    }

    #[test]
    fn oplus_assign_is_addition_for_a_euclidean_block() {
        // A position block is Euclidean, so `oplus_assign` reduces to `mean += delta`.
        let mut s = FrameAwareState::from_schema(
            Arc::new(StateSchema::compose(vec![StateSchemaBlock::new(
                Quantity::Position(FrameId::world()),
                Convention::Enu,
                None,
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            )])),
            MonotonicTime(0.0),
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::world()), Component::X),
            1.0,
        );
        s.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::world()), Component::Y),
            2.0,
        );

        s.oplus_assign(&DVector::from_vec(vec![0.5, -0.5, 0.0]));
        assert_eq!(s.mean, DVector::from_vec(vec![1.5, 1.5, 0.0]));
    }

    #[test]
    fn from_schema_takes_initial_value() {
        let schema = pose_schema();
        let s = FrameAwareState::from_schema(schema.clone(), MonotonicTime(5.0));

        assert_eq!(&s.mean, schema.initial_value());
        assert_eq!(s.timestamp, MonotonicTime(5.0));
    }
}
