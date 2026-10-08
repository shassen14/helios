//! The filter component: the recursive algorithm a node runs (EKF, …), built
//! around a seeded state and a dynamics model.

use crate::assembly::{BuildContext, NoParams};
use crate::nodes::estimation::EstimatorComponents;

use helios_core::estimation::dynamics::EstimationDynamics;
use helios_core::estimation::filters::ekf::ExtendedKalmanFilter;
use helios_core::estimation::GaussianStateEstimator;
use helios_core::spatial::FrameAwareState;

use nalgebra::DMatrix;

/// The `kind` a `filter` sub-table writes for the extended Kalman filter.
pub(crate) const EKF_FILTER_KIND: &str = "Ekf";

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

pub(super) fn register(components: &mut EstimatorComponents) {
    components
        .register_filter(EKF_FILTER_KIND, build_ekf)
        .expect("Ekf is registered once, by the default components");
}

fn build_ekf(
    _config: NoParams,
    _ctx: &BuildContext<'_>,
    parts: FilterParts,
) -> Result<Box<dyn GaussianStateEstimator>, String> {
    Ok(Box::new(ExtendedKalmanFilter::new(
        parts.initial_state,
        parts.process_noise,
        parts.dynamics,
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::{AutonomyRegistry, ComponentError};

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

    fn build(
        registry: &AutonomyRegistry,
        section: &str,
    ) -> Result<Box<dyn GaussianStateEstimator>, ComponentError> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(agent(), NODE, &channels);
        registry
            .extension::<EstimatorComponents>()
            .expect("the default registry has estimator components")
            .build_filter("filter", section, &ctx, parts())
            .map(|built| built.component)
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
    fn the_built_in_ekf_takes_no_parameters() {
        let registry = AutonomyRegistry::default();
        let Err(err) = build(&registry, "kind = \"Ekf\"\nalpha = 1e-3") else {
            panic!("a parameter on the EKF must not build");
        };
        assert!(matches!(err, ComponentError::InvalidConfig { .. }), "{err}");
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
        build_ekf(NoParams {}, _ctx, parts)
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
