//! State schemas for producers that *republish a known state* rather than filter
//! it: a mock estimator, a ground-truth publisher, a replay source.
//!
//! A carrier does not run a predict step — it already knows the truth and only
//! needs to express it. So a carrier schema is the composed block shape a
//! consumer reads from, paired with a *certain* prior (zero `P₀`): the block
//! identities matter, the covariance does not.
//!
//! This is model-agnostic on purpose. The shape here is not any one estimator's
//! output; it is the kinematic state a truth source can express, so swapping the
//! stack's real estimator never touches it. It composes generic [`Quantity`]
//! blocks — the same vocabulary a filtering schema uses — through
//! [`StateSchema::compose`]; it is a *client* of that primitive, not a new one,
//! which is why it is a free function and not a `StateSchema` constructor.

use crate::data::primitives::FrameHandle;
use crate::estimation::schema::{Quantity, SchemaBlock, StateSchema};
use crate::frames::FrameId;
use crate::manifold::TangentNoise;

use nalgebra::{DMatrix, DVector};

/// A never-read process-noise variance for a carrier's orientation block. A
/// carrier never predicts, so no `Q` value here is ever consumed — but an
/// orientation block cannot be built from zero (non-positive-definite) noise, so
/// the one curved block is given a small positive variance purely to satisfy
/// that construction invariant. Flat blocks take `None` and carry none.
const CARRIER_ORIENTATION_NOISE_VAR: f64 = 1.0;

/// The kinematic state a pose + twist reference source expresses: odom-frame
/// position, linear velocity, and angular velocity, plus the body → odom
/// attitude. Every block is composed by [`Quantity`], so a consumer reads it by
/// typed block extractor exactly as it would a filtering estimate.
///
/// It carries no sensor-bias or acceleration blocks — a reference source has no
/// biases and a pose + twist source measures no acceleration. Angular velocity
/// is in the odom (ENU) frame, matching how the estimate reports it.
pub fn kinematic_carrier_schema(agent_handle: FrameHandle) -> StateSchema {
    let body = FrameId::Body(agent_handle);
    let odom = FrameId::Odom(agent_handle);

    // A flat, certain block: no process noise, zero initial covariance.
    let flat = |quantity: Quantity| {
        SchemaBlock::new(quantity, None, DVector::zeros(3), DMatrix::zeros(3, 3))
    };

    StateSchema::compose(vec![
        flat(Quantity::Position(odom.clone())),
        flat(Quantity::Velocity(odom.clone())),
        SchemaBlock::new(
            Quantity::Orientation {
                from: body,
                to: odom.clone(),
            },
            Some(
                TangentNoise::from_variances(DVector::from_element(
                    3,
                    CARRIER_ORIENTATION_NOISE_VAR,
                ))
                .expect("positive variance is positive-definite"),
            ),
            DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
            DMatrix::zeros(3, 3),
        ),
        flat(Quantity::AngularVelocity(odom)),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::conventions::{Enu, Flu};
    use crate::frames::{FrameAwareState, StateVariable};
    use std::sync::Arc;

    const AGENT: FrameHandle = FrameHandle(1);

    fn carrier() -> FrameAwareState {
        FrameAwareState::from_schema(Arc::new(kinematic_carrier_schema(AGENT)), 0.0)
    }

    #[test]
    fn carries_the_four_kinematic_blocks() {
        let s = carrier();
        // Storage: 3 (pos) + 3 (vel) + 4 (quat) + 3 (ang-vel) = 13.
        assert_eq!(s.storage_dim(), 13);
        // Tangent: the quaternion spends 3, not 4 → 3 + 3 + 3 + 3 = 12.
        assert_eq!(s.tangent_dim(), 12);
    }

    #[test]
    fn seeds_an_identity_attitude_and_a_certain_prior() {
        let s = carrier();
        // The orientation block seeds the identity quaternion, readable back as
        // the identity rotation.
        let r = s
            .orientation::<Flu, Enu>(FrameId::Body(AGENT), FrameId::Odom(AGENT))
            .expect("carrier holds the attitude block");
        assert!((r.into_inner().angle()).abs() < 1e-12);
        // A carrier is certain: its covariance is all zeros.
        assert!(s.covariance.iter().all(|&c| c == 0.0));
    }

    #[test]
    fn every_kinematic_quantity_reads_back() {
        let mut s = carrier();
        s.set_variable(&StateVariable::Vx(FrameId::Odom(AGENT)), 2.0);
        s.set_variable(&StateVariable::Wz(FrameId::Odom(AGENT)), 0.3);

        assert_eq!(
            s.velocity::<Enu>(FrameId::Odom(AGENT)).unwrap().x(),
            2.0,
            "odom linear velocity reads back"
        );
        assert_eq!(
            s.angular_velocity::<Enu>(FrameId::Odom(AGENT)).unwrap().z(),
            0.3,
            "odom angular velocity reads back"
        );
        assert!(
            s.position::<Enu>(FrameId::Odom(AGENT)).is_some(),
            "odom position block is present"
        );
    }
}
