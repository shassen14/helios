//! State-augmentation blocks: extra estimated quantities appended to a base
//! state so the filter solves for them online.
//!
//! The base state carries the physically shared quantities (pose, velocity).
//! An augmentation block carries a *per-device nuisance parameter* — a sensor's
//! own error terms — that the filter estimates alongside the trajectory instead
//! of trusting a fixed factory value. Estimating those terms while the vehicle
//! runs is online calibration; the same block, frozen after convergence, is the
//! offline-calibration result. Each block is tied to the sensor frame it
//! describes (carried in its [`Quantity::MagBias`] identity), so two
//! magnetometers on one vehicle get two independent bias blocks that never alias.
//!
//! [`augmentation_block`] maps a config-declared augmentation kind to the
//! [`SchemaBlock`] that realizes it, ready for [`StateSchema::compose`] to bake
//! into the estimator's state.
//!
//! [`StateSchema::compose`]: crate::estimation::schema::StateSchema::compose

use crate::{
    estimation::schema::SchemaBlock,
    frames::{transforms::Convention, FrameId},
    manifold::TangentNoise,
    state::Quantity,
};

use std::fmt::Display;

use nalgebra::{DMatrix, DVector};

pub const MAGNETOMETER_BIAS: &str = "magnetometer_bias";

/// Why an augmentation kind could not be turned into a [`SchemaBlock`].
///
/// Both variants are load-time configuration faults surfaced to the caller so
/// the offending field can be named; neither occurs on a well-formed config.
#[derive(Debug)]
pub enum AugmentationError {
    /// The `kind` string matched no known augmentation. Carries the string so
    /// the load error can echo it back.
    UnknownKind(String),
    /// The noise parameters did not form a valid (positive-definite) covariance
    /// — e.g. a zero or non-finite random-walk. `TangentNoise` rejected them.
    InvalidNoise,
}

impl Display for AugmentationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownKind(kind) => {
                write!(
                    f,
                    "unknown augmentation kind '{kind}'; no such block is registered"
                )
            }
            Self::InvalidNoise => write!(
                f,
                "invalid augmentation noise: random-walk must be positive and finite"
            ),
        }
    }
}

/// Builds the [`SchemaBlock`] for one augmentation `kind`, tied to `sensor`.
///
/// * `kind` — the augmentation to instantiate; matched against the reserved kind
///   strings (e.g. [`MAGNETOMETER_BIAS`]). An unrecognized value is a
///   [`AugmentationError::UnknownKind`], not a panic.
/// * `sensor` — the frame the estimated parameter belongs to. It is carried in the
///   block's [`Quantity`] identity, from which every [`StateVariable`] name is
///   derived tagged with it, so blocks for distinct sensors stay independent.
/// * `init_uncertainty` — prior standard deviation on each axis (block units).
///   Squared onto the diagonal of the block's initial covariance `P₀`.
/// * `random_walk` — per-axis process-noise standard deviation driving how fast
///   the estimate may drift; forms the block's `Q`.
///
/// The magnetometer-bias block is a 3-DOF Euclidean block over
/// `MagBias{X,Y,Z}`, isotropic in both `P₀` and `Q`.
pub fn augmentation_block(
    kind: &str,
    sensor: FrameId,
    init_uncertainty: f64,
    random_walk: f64,
) -> Result<SchemaBlock, AugmentationError> {
    match kind {
        MAGNETOMETER_BIAS => {
            let noise = TangentNoise::from_std_devs(DVector::from_element(3, random_walk))
                .ok_or(AugmentationError::InvalidNoise)?;

            let p0 = DMatrix::from_diagonal(&DVector::from_element(
                3,
                init_uncertainty * init_uncertainty,
            ));

            Ok(SchemaBlock::new(
                Quantity::MagBias(sensor),
                Convention::Flu,
                Some(noise),
                DVector::zeros(3),
                p0,
            ))
        }
        _ => Err(AugmentationError::UnknownKind(kind.to_owned())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::frames::StateVariable;
    use crate::state::Component;

    const INIT_UNCERTAINTY: f64 = 0.5;
    const RANDOM_WALK: f64 = 0.01;

    fn sensor_frame() -> FrameId {
        FrameId::Sensor(FrameHandle(7))
    }

    fn mag_block() -> SchemaBlock {
        augmentation_block(
            MAGNETOMETER_BIAS,
            sensor_frame(),
            INIT_UNCERTAINTY,
            RANDOM_WALK,
        )
        .expect("well-formed magnetometer-bias request should build")
    }

    #[test]
    fn mag_bias_variables_are_tagged_with_the_sensor_frame() {
        let block = mag_block();
        assert_eq!(
            block.variables(),
            vec![
                StateVariable::new(Quantity::MagBias(sensor_frame()), Component::X),
                StateVariable::new(Quantity::MagBias(sensor_frame()), Component::Y),
                StateVariable::new(Quantity::MagBias(sensor_frame()), Component::Z),
            ]
        );
    }

    #[test]
    fn mag_bias_records_its_owning_sensor() {
        assert_eq!(mag_block().quantity, Quantity::MagBias(sensor_frame()));
    }

    #[test]
    fn mag_bias_is_a_three_dof_block() {
        let block = mag_block();
        assert_eq!(block.block.storage_dim(), 3);
        assert_eq!(block.block.tangent_dim(), 3);
        assert_eq!(block.variables().len(), block.block.storage_dim());
    }

    #[test]
    fn mag_bias_prior_mean_is_zero() {
        assert_eq!(mag_block().block.initial_value(), DVector::zeros(3));
    }

    #[test]
    fn mag_bias_initial_covariance_is_init_uncertainty_squared_on_the_diagonal() {
        let expected = DMatrix::from_diagonal(&DVector::from_element(
            3,
            INIT_UNCERTAINTY * INIT_UNCERTAINTY,
        ));
        assert_eq!(mag_block().block.initial_covariance(), expected);
    }

    #[test]
    fn mag_bias_process_noise_is_random_walk_squared_on_the_diagonal() {
        let noise = mag_block()
            .block
            .process_noise()
            .expect("a random-walk bias block must carry process noise");
        let expected = DMatrix::from_diagonal(&DVector::from_element(3, RANDOM_WALK * RANDOM_WALK));
        assert_eq!(noise.covariance(), &expected);
    }

    #[test]
    fn unknown_kind_is_reported_with_the_offending_string() {
        match augmentation_block(
            "gyro_scale_factor",
            sensor_frame(),
            INIT_UNCERTAINTY,
            RANDOM_WALK,
        ) {
            Err(AugmentationError::UnknownKind(kind)) => assert_eq!(kind, "gyro_scale_factor"),
            other => panic!("expected UnknownKind, got {other:?}"),
        }
    }

    #[test]
    fn zero_random_walk_is_rejected_as_invalid_noise() {
        // A zero variance yields a singular (non-positive-definite) covariance,
        // which TangentNoise's Cholesky refuses.
        let result = augmentation_block(MAGNETOMETER_BIAS, sensor_frame(), INIT_UNCERTAINTY, 0.0);
        assert!(matches!(result, Err(AugmentationError::InvalidNoise)));
    }
}
