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
//! [`AugmentationSpec`] checks a config-declared augmentation kind and its noise
//! without a sensor, then yields the [`StateSchemaBlock`] that realizes it for
//! each sensor frame, ready for [`StateSchema::compose`] to bake into the
//! estimator's state. [`augmentation_block`] does both at once.
//!
//! [`StateSchema::compose`]: crate::estimation::schema::StateSchema::compose

use crate::{
    estimation::schema::StateSchemaBlock,
    kernel::manifold::TangentNoise,
    spatial::state::Quantity,
    spatial::{transforms::Convention, FrameId},
};

use std::fmt::Display;

use nalgebra::{DMatrix, DVector};

pub const MAGNETOMETER_BIAS: &str = "magnetometer_bias";

/// A magnetometer bias has one component per field axis.
const MAGNETOMETER_BIAS_DIM: usize = 3;

/// Why an augmentation's config is unusable.
///
/// Every variant is a load-time configuration fault surfaced to the caller so
/// the offending field can be named; none occurs on a well-formed config.
#[derive(Debug)]
pub enum AugmentationError {
    /// The `kind` string matched no known augmentation. Carries the string so
    /// the load error can echo it back.
    UnknownKind(String),
    /// The noise parameters did not form a valid (positive-definite) covariance
    /// — e.g. a zero or non-finite random-walk. `TangentNoise` rejected them.
    InvalidNoise,
    /// The initial uncertainty was not finite and positive. A zero gives a
    /// singular `P₀`, so the block is never corrected; a non-finite one puts
    /// NaN into the filter. Carries the value so the load error can echo it.
    InvalidUncertainty(f64),
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
            Self::InvalidUncertainty(value) => write!(
                f,
                "invalid augmentation init_uncertainty: must be finite and positive, got {value}"
            ),
        }
    }
}

/// One augmentation, checked: a known kind with a usable noise model, not yet
/// tied to a sensor frame.
///
/// Built from config before the sensor frames are known, so a bad kind or noise
/// is refused early; [`block`](Self::block) then ties it to its sensor.
#[derive(Debug)]
pub struct AugmentationSpec {
    kind: AugmentationKind,
    init_uncertainty: f64,
    noise: TangentNoise,
}

impl AugmentationSpec {
    /// Checks one augmentation `kind` and its noise.
    ///
    /// * `kind` — the augmentation to instantiate; matched against the reserved
    ///   kind strings (e.g. [`MAGNETOMETER_BIAS`]). An unrecognized value is an
    ///   [`AugmentationError::UnknownKind`], not a panic.
    /// * `init_uncertainty` — prior standard deviation on each axis (block
    ///   units). Squared onto the diagonal of the block's initial covariance `P₀`.
    ///   A value not finite and positive is an
    ///   [`AugmentationError::InvalidUncertainty`].
    /// * `random_walk` — per-axis process-noise standard deviation driving how
    ///   fast the estimate may drift; forms the block's `Q`. A value that gives
    ///   no positive-definite covariance is an [`AugmentationError::InvalidNoise`].
    pub fn new(
        kind: &str,
        init_uncertainty: f64,
        random_walk: f64,
    ) -> Result<Self, AugmentationError> {
        let kind = match kind {
            MAGNETOMETER_BIAS => AugmentationKind::MagnetometerBias,
            _ => return Err(AugmentationError::UnknownKind(kind.to_owned())),
        };
        if !init_uncertainty.is_finite() || init_uncertainty <= 0.0 {
            return Err(AugmentationError::InvalidUncertainty(init_uncertainty));
        }
        let noise = TangentNoise::from_std_devs(DVector::from_element(kind.dim(), random_walk))
            .ok_or(AugmentationError::InvalidNoise)?;

        Ok(Self {
            kind,
            init_uncertainty,
            noise,
        })
    }

    /// The [`StateSchemaBlock`] for this augmentation, tied to `sensor`.
    ///
    /// `sensor` is the frame the estimated parameter belongs to. It is carried in
    /// the block's [`Quantity`] identity, from which every [`StateVariable`] name
    /// is derived tagged with it, so blocks for distinct sensors stay
    /// independent.
    ///
    /// The magnetometer-bias block is a 3-DOF Euclidean block over
    /// `MagBias{X,Y,Z}`, isotropic in both `P₀` and `Q`.
    ///
    /// [`StateVariable`]: crate::spatial::StateVariable
    pub fn block(&self, sensor: FrameId) -> StateSchemaBlock {
        let dim = self.kind.dim();
        let p0 = DMatrix::from_diagonal(&DVector::from_element(
            dim,
            self.init_uncertainty * self.init_uncertainty,
        ));

        match self.kind {
            AugmentationKind::MagnetometerBias => StateSchemaBlock::new(
                Quantity::MagBias(sensor),
                Convention::Flu,
                Some(self.noise.clone()),
                DVector::zeros(dim),
                p0,
            ),
        }
    }
}

/// The augmentations [`AugmentationSpec`] knows, one per reserved kind string.
#[derive(Clone, Copy, Debug)]
enum AugmentationKind {
    MagnetometerBias,
}

impl AugmentationKind {
    /// The block's degrees of freedom.
    fn dim(self) -> usize {
        match self {
            Self::MagnetometerBias => MAGNETOMETER_BIAS_DIM,
        }
    }
}

/// Builds the [`StateSchemaBlock`] for one augmentation `kind`, tied to
/// `sensor`: [`AugmentationSpec::new`], then [`AugmentationSpec::block`].
pub fn augmentation_block(
    kind: &str,
    sensor: FrameId,
    init_uncertainty: f64,
    random_walk: f64,
) -> Result<StateSchemaBlock, AugmentationError> {
    Ok(AugmentationSpec::new(kind, init_uncertainty, random_walk)?.block(sensor))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::AgentId;
    use crate::spatial::state::Component;
    use crate::spatial::StateVariable;

    const INIT_UNCERTAINTY: f64 = 0.5;
    const RANDOM_WALK: f64 = 0.01;

    fn sensor_frame() -> FrameId {
        FrameId::agent(AgentId::new("test_agent"), "sensor0")
    }

    fn mag_block() -> StateSchemaBlock {
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

    /// A spec is checked without a sensor: an unknown kind is refused before
    /// any frame exists.
    #[test]
    fn a_spec_refuses_an_unknown_kind() {
        match AugmentationSpec::new("gyro_scale_factor", INIT_UNCERTAINTY, RANDOM_WALK) {
            Err(AugmentationError::UnknownKind(kind)) => assert_eq!(kind, "gyro_scale_factor"),
            other => panic!("expected UnknownKind, got {other:?}"),
        }
    }

    #[test]
    fn a_spec_refuses_invalid_noise() {
        for random_walk in [0.0, f64::NAN, f64::INFINITY] {
            let result = AugmentationSpec::new(MAGNETOMETER_BIAS, INIT_UNCERTAINTY, random_walk);
            assert!(
                matches!(result, Err(AugmentationError::InvalidNoise)),
                "random_walk = {random_walk}: {result:?}"
            );
        }
    }

    #[test]
    fn a_spec_refuses_an_unusable_init_uncertainty() {
        for init_uncertainty in [0.0, -0.5, f64::NAN, f64::INFINITY] {
            let result = AugmentationSpec::new(MAGNETOMETER_BIAS, init_uncertainty, RANDOM_WALK);
            assert!(
                matches!(result, Err(AugmentationError::InvalidUncertainty(_))),
                "init_uncertainty = {init_uncertainty}: {result:?}"
            );
        }
    }

    /// One spec yields one block per sensor: distinct variables, the same
    /// prior and process noise.
    #[test]
    fn one_spec_yields_an_independent_block_per_sensor() {
        let spec = AugmentationSpec::new(MAGNETOMETER_BIAS, INIT_UNCERTAINTY, RANDOM_WALK)
            .expect("well-formed magnetometer-bias spec");
        let other = FrameId::agent(AgentId::new("test_agent"), "sensor1");

        let first = spec.block(sensor_frame());
        let second = spec.block(other.clone());

        assert_eq!(first.quantity, Quantity::MagBias(sensor_frame()));
        assert_eq!(second.quantity, Quantity::MagBias(other));
        assert!(first
            .variables()
            .iter()
            .all(|variable| !second.variables().contains(variable)));
        assert_eq!(
            first.block.initial_covariance(),
            second.block.initial_covariance()
        );
        assert_eq!(
            first
                .block
                .process_noise()
                .map(|noise| noise.covariance().clone()),
            second
                .block
                .process_noise()
                .map(|noise| noise.covariance().clone())
        );
    }
}
