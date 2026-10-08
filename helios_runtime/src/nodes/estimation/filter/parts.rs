//! [`FilterParts`], the built-in filter kinds and their factories.

use super::config::{EkfConfig, UkfConfig};

use crate::assembly::BuildContext;
use crate::nodes::estimation::EstimatorComponents;

use helios_core::estimation::dynamics::EstimationDynamics;
use helios_core::estimation::filters::ekf::ExtendedKalmanFilter;
use helios_core::estimation::filters::ukf::UnscentedKalmanFilter;
use helios_core::estimation::GaussianStateEstimator;
use helios_core::spatial::FrameAwareState;

use nalgebra::DMatrix;

/// The `kind` a `filter` sub-table writes for the extended Kalman filter.
pub(crate) const EKF_FILTER_KIND: &str = "Ekf";
/// The `kind` a `filter` sub-table writes for the unscented Kalman filter.
pub(crate) const UKF_FILTER_KIND: &str = "Ukf";

/// What a filter factory receives besides its own config: the parts the node
/// built before choosing the filter.
///
/// Built only by the node, so the three always agree in size: Q is read from
/// the state's schema, and the state is laid out by the dynamics' schema plus
/// any augmentation blocks.
#[non_exhaustive]
pub struct FilterParts {
    /// The prior: mean, covariance P₀ and valid-at time, on the composed
    /// schema.
    pub initial_state: FrameAwareState,
    /// The process noise Q, tangent × tangent on the same schema.
    pub process_noise: DMatrix<f64>,
    /// The process model.
    pub dynamics: Box<dyn EstimationDynamics>,
}

impl FilterParts {
    /// Parts for `initial_state` and `dynamics`, with Q read from the state's
    /// schema.
    pub(crate) fn new(
        initial_state: FrameAwareState,
        dynamics: Box<dyn EstimationDynamics>,
    ) -> Self {
        let process_noise = initial_state.schema().process_noise().clone();
        Self {
            initial_state,
            process_noise,
            dynamics,
        }
    }
}

pub(in crate::nodes::estimation) fn register(components: &mut EstimatorComponents) {
    components
        .register_filter(EKF_FILTER_KIND, build_ekf)
        .expect("Ekf is registered once, by the default components");
    components
        .register_filter(UKF_FILTER_KIND, build_ukf)
        .expect("Ukf is registered once, by the default components");
}

fn build_ekf(
    params: EkfConfig,
    _ctx: &BuildContext<'_>,
    parts: FilterParts,
) -> Result<Box<dyn GaussianStateEstimator>, String> {
    let conditioning = params.conditioning()?;
    Ok(Box::new(
        ExtendedKalmanFilter::new(parts.initial_state, parts.process_noise, parts.dynamics)
            .with_conditioning(conditioning),
    ))
}

/// Builds a UKF, refusing a state with a curved block.
///
/// The UKF's predicted mean is a plain weighted sum of the sigma points. That
/// is the mean only on a flat block; on a rotation it leaves the manifold
/// (a sum of unit quaternions is not one). Until the mean is computed on the
/// manifold, a curved block is refused here rather than estimated wrongly.
fn build_ukf(
    params: UkfConfig,
    _ctx: &BuildContext<'_>,
    parts: FilterParts,
) -> Result<Box<dyn GaussianStateEstimator>, String> {
    let schema = parts.initial_state.schema();
    if let Some(curved) = schema.blocks().iter().find(|block| !block.is_euclidean()) {
        return Err(format!(
            "the UKF needs every state block to be Euclidean, but {} is curved: its \
             sigma-point mean is a plain weighted sum, which is not a mean on a rotation",
            curved.quantity()
        ));
    }
    let params = params.for_dof(schema.tangent_dim())?;
    Ok(Box::new(UnscentedKalmanFilter::new(
        parts.initial_state,
        parts.process_noise,
        parts.dynamics,
        params,
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::{AutonomyRegistry, ComponentError};
    use crate::nodes::estimation::flat_position::FlatPosition;

    use helios_core::estimation::dynamics::integrated_imu::{
        ImuInitialUncertainty, ImuProcessNoise, IntegratedImuModel,
    };
    use helios_core::estimation::{EstimatorInputs, PredictOutcome};
    use helios_core::prelude::{AgentId, MonotonicTime};

    use nalgebra::{DVector, Vector3};
    use serde::{Deserialize, Serialize};
    use std::collections::HashSet;

    const NODE: &str = "primary";
    const DUMMY_KIND: &str = "FrozenEkf";
    const DT: f64 = 0.02;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn parts() -> FilterParts {
        let dynamics = IntegratedImuModel::new(
            agent(),
            Vector3::new(0.0, 0.0, -9.81),
            ImuProcessNoise {
                accel_noise_var: 0.04,
                gyro_noise_var: 0.0025,
                accel_bias_var: 0.0001,
                gyro_bias_var: 0.000001,
            },
            ImuInitialUncertainty {
                pos_var: 0.5,
                vel_var: 1.0,
                ori_var: 0.02,
                accel_bias_var: 1.0,
                gyro_bias_var: 1.0,
            },
        );
        let state = FrameAwareState::from_schema(dynamics.schema(), MonotonicTime(0.0));
        FilterParts::new(state, Box::new(dynamics))
    }

    /// Gravity-compensated, otherwise-still IMU input.
    fn still() -> EstimatorInputs {
        EstimatorInputs {
            control: DVector::from_row_slice(&[0.0, 0.0, 9.81, 0.0, 0.0, 0.0]),
        }
    }

    /// Parts on a flat state: a still `odom` position.
    fn flat_parts() -> FilterParts {
        let dynamics = FlatPosition::new(agent());
        let state = FrameAwareState::from_schema(dynamics.schema(), MonotonicTime(0.0));
        FilterParts::new(state, Box::new(dynamics))
    }

    fn build(
        registry: &AutonomyRegistry,
        section: &str,
    ) -> Result<Box<dyn GaussianStateEstimator>, ComponentError> {
        build_on(registry, section, parts())
    }

    fn build_on(
        registry: &AutonomyRegistry,
        section: &str,
        parts: FilterParts,
    ) -> Result<Box<dyn GaussianStateEstimator>, ComponentError> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(agent(), NODE, &channels);
        registry
            .extension::<EstimatorComponents>()
            .expect("the default registry has estimator components")
            .build_filter("filter", section, &ctx, parts)
            .map(|built| built.component)
    }

    /// A UKF section with the usual Gaussian spread.
    fn ukf_section() -> String {
        format!("kind = \"{UKF_FILTER_KIND}\"\nalpha = 1e-3\nbeta = 2.0\nkappa = 0.0")
    }

    /// Q is read from the state's schema, so it is sized to the state.
    #[test]
    fn parts_read_q_from_the_state_schema() {
        let parts = parts();
        assert_eq!(
            &parts.process_noise,
            parts.initial_state.schema().process_noise()
        );
    }

    /// The built-in EKF builds from the parts and predicts.
    #[test]
    fn the_built_in_ekf_builds_and_predicts() {
        let registry = AutonomyRegistry::default();
        let Ok(mut filter) = build(&registry, &format!("kind = \"{EKF_FILTER_KIND}\"")) else {
            panic!("the built-in EKF builds");
        };
        assert_eq!(filter.predict(DT, &still()), PredictOutcome::Applied);
        assert_eq!(filter.state().timestamp, MonotonicTime(DT));
    }

    #[test]
    fn the_built_in_ekf_rejects_an_unknown_parameter() {
        let registry = AutonomyRegistry::default();
        let Err(err) = build(&registry, "kind = \"Ekf\"\nalpha = 1e-3") else {
            panic!("an unknown parameter on the EKF must not build");
        };
        assert!(matches!(err, ComponentError::InvalidConfig { .. }), "{err}");
    }

    #[test]
    fn the_built_in_ekf_takes_a_floor_and_a_jitter() {
        let registry = AutonomyRegistry::default();
        let section = "kind = \"Ekf\"\ncovariance_floor = 1e-9\ncovariance_jitter = 1e-12";
        assert!(build(&registry, section).is_ok());
    }

    #[test]
    fn a_non_positive_or_non_finite_conditioning_value_is_rejected() {
        let registry = AutonomyRegistry::default();
        for value in ["0.0", "-1e-9", "nan", "inf"] {
            for key in ["covariance_floor", "covariance_jitter"] {
                let section = format!("kind = \"Ekf\"\n{key} = {value}");
                let Err(err) = build(&registry, &section) else {
                    panic!("{key} = {value} must not build");
                };
                assert!(matches!(err, ComponentError::BuildFailed { .. }), "{err}");
            }
        }
    }

    /// The built-in UKF builds on a flat state and predicts: P grows by Q·dt.
    #[test]
    fn the_built_in_ukf_builds_on_a_flat_state_and_predicts() {
        let registry = AutonomyRegistry::default();
        let parts = flat_parts();
        let p0 = parts.initial_state.covariance.clone();
        let q = parts.process_noise.clone();
        let Ok(mut filter) = build_on(&registry, &ukf_section(), parts) else {
            panic!("the built-in UKF builds on a flat state");
        };
        let none = EstimatorInputs {
            control: DVector::zeros(0),
        };

        assert_eq!(filter.predict(DT, &none), PredictOutcome::Applied);
        assert_eq!(filter.state().timestamp, MonotonicTime(DT));
        let expected = p0 + q * DT;
        assert!(
            (&filter.state().covariance - &expected).amax() < 1e-9,
            "P after predict: {}, expected {expected}",
            filter.state().covariance
        );
    }

    /// The INS state holds an orientation, which the UKF's mean can't average.
    #[test]
    fn the_ukf_refuses_a_state_with_a_curved_block() {
        let registry = AutonomyRegistry::default();
        let Err(err) = build(&registry, &ukf_section()) else {
            panic!("the UKF must not build on a state holding an orientation");
        };
        assert!(matches!(err, ComponentError::BuildFailed { .. }), "{err}");
        let message = err.to_string();
        assert!(message.contains("Euclidean"), "{message}");
        assert!(message.contains("orientation"), "{message}");
    }

    #[test]
    fn the_ukf_requires_its_spread_and_rejects_unknown_keys() {
        let registry = AutonomyRegistry::default();
        for section in [
            format!("kind = \"{UKF_FILTER_KIND}\"\nalpha = 1e-3\nbeta = 2.0"),
            format!("{}\ncovariance_floor = 1e-9", ukf_section()),
        ] {
            let Err(err) = build_on(&registry, &section, flat_parts()) else {
                panic!("`{section}` must not build");
            };
            assert!(matches!(err, ComponentError::InvalidConfig { .. }), "{err}");
        }
    }

    #[test]
    fn an_unusable_ukf_spread_is_rejected() {
        let registry = AutonomyRegistry::default();
        // The flat state has 3 degrees of freedom, so kappa must exceed -3.
        for (alpha, beta, kappa) in [
            ("0.0", "2.0", "0.0"),
            ("-1e-3", "2.0", "0.0"),
            ("nan", "2.0", "0.0"),
            ("1e-3", "inf", "0.0"),
            ("1e-3", "2.0", "nan"),
            ("1e-3", "2.0", "-3.0"),
        ] {
            let section = format!(
                "kind = \"{UKF_FILTER_KIND}\"\nalpha = {alpha}\nbeta = {beta}\nkappa = {kappa}"
            );
            let Err(err) = build_on(&registry, &section, flat_parts()) else {
                panic!("alpha = {alpha}, beta = {beta}, kappa = {kappa} must not build");
            };
            assert!(matches!(err, ComponentError::BuildFailed { .. }), "{err}");
        }
    }

    /// A dummy filter kind with its own strictly-parsed parameter, registered
    /// from outside the built-ins and standing in for a researcher's filter:
    /// the EKF under another name, which refuses `frozen = false`.
    #[derive(Deserialize, Serialize)]
    #[serde(deny_unknown_fields)]
    struct FrozenConfig {
        frozen: bool,
    }

    fn build_frozen(
        config: FrozenConfig,
        _ctx: &BuildContext<'_>,
        parts: FilterParts,
    ) -> Result<Box<dyn GaussianStateEstimator>, String> {
        if !config.frozen {
            return Err("only the frozen variant exists".to_string());
        }
        build_ekf(EkfConfig::default(), _ctx, parts)
    }

    #[test]
    fn a_registered_filter_kind_builds_with_its_own_params() {
        let mut registry = AutonomyRegistry::default();
        registry
            .extension_mut::<EstimatorComponents>()
            .register_filter(DUMMY_KIND, build_frozen)
            .expect("not a built-in kind");

        assert!(build(
            &registry,
            &format!("kind = \"{DUMMY_KIND}\"\nfrozen = true")
        )
        .is_ok());
        let Err(err) = build(
            &registry,
            &format!("kind = \"{DUMMY_KIND}\"\nfrozen = false"),
        ) else {
            panic!("the factory's own rejection surfaces");
        };
        assert!(matches!(err, ComponentError::BuildFailed { .. }), "{err}");
    }

    #[test]
    fn registering_a_built_in_filter_kind_again_is_rejected() {
        let mut registry = AutonomyRegistry::default();
        let err = registry
            .extension_mut::<EstimatorComponents>()
            .register_filter(EKF_FILTER_KIND, build_frozen)
            .expect_err("Ekf is a built-in");
        assert_eq!(err.to_string(), "filter kind 'Ekf' is already registered");
    }
}
