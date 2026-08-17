//! Registers Gaussian filter factories (EKF, UKF).

use super::input::IntegratedImuInputBuilder;
use super::node::GaussianEstimatorNode;

use crate::config::{EkfDynamicsConfig, EstimatorConfig};
use crate::nodes::gaussian_estimator::EstimatorInputBuilder;
use crate::pipeline::node::PipelineNode;
use crate::registry::{contexts::GaussianEstimatorBuildContext, AutonomyRegistry};

use helios_core::estimation::dynamics::{integrated_imu::IntegratedImuModel, EstimationDynamics};
use helios_core::estimation::filters::ekf::ExtendedKalmanFilter;
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_gaussian_estimator("Ekf", build_ekf);
    registry
        .register_gaussian_estimator("Ukf", |_, _, _| Err("UKF not yet implemented".to_string()));
}

fn build_ekf(
    config: EstimatorConfig,
    ctx: GaussianEstimatorBuildContext,
    _registry: &AutonomyRegistry,
) -> Result<Box<dyn PipelineNode>, String> {
    let EstimatorConfig::Ekf(ekf_config) = config else {
        return Err("build_ekf received non-Ekf config".to_string());
    };

    let agent_handle = ctx.agent_handle;

    let init = &ekf_config.initial_state;

    let (dynamics, input_builder): (Box<dyn EstimationDynamics>, Box<dyn EstimatorInputBuilder>) =
        match &ekf_config.dynamics {
            EkfDynamicsConfig::IntegratedImu(c) => (
                Box::new(IntegratedImuModel::new(
                    agent_handle,
                    Vector3::from_column_slice(&c.gravity_enu),
                    c.accel_noise_stddev.powi(2),
                    c.gyro_noise_stddev.powi(2),
                    c.accel_bias_instability.powi(2),
                    c.gyro_bias_instability.powi(2),
                    init.position_uncertainty_m.powi(2),
                    init.orientation_uncertainty_deg.to_radians().powi(2),
                )),
                Box::new(IntegratedImuInputBuilder::new(
                    c.accel_channel.as_str(),
                    c.gyro_channel.as_str(),
                )),
            ),
            EkfDynamicsConfig::AckermannOdometry(_) | EkfDynamicsConfig::Quadcopter(_) => {
                return Err(format!(
                    "EKF dynamics kind '{}' is not yet implemented",
                    ekf_config.dynamics.get_kind_str()
                ));
            }
        };

    // The dynamics model is the single author of its state shape: process
    // noise (Q) and initial covariance (P₀) are both baked into the schema from
    // the variances passed to `new` above, so we read them back rather than
    // rebuilding them here.
    let schema = dynamics.schema();
    let q = schema.process_noise().clone();

    // Seed the initial state from the schema (mean = zeros + identity
    // orientation, covariance = P₀), then overwrite only the mean pose with this
    // scenario's starting position/heading — the one thing the schema can't know.
    let mut initial_state = FrameAwareState::from_schema(schema, 0.0);

    let yaw = init.heading_deg.to_radians();
    let iso = Isometry3::from_parts(
        Translation3::new(init.x, init.y, init.z),
        UnitQuaternion::from_quaternion(Quaternion::new(
            (yaw / 2.0).cos(),
            0.0,
            0.0,
            (yaw / 2.0).sin(),
        )),
    );

    let body = FrameId::Body(agent_handle);
    let world = FrameId::World;

    initial_state.set_variable(&StateVariable::Px(FrameId::World), iso.translation.x);
    initial_state.set_variable(&StateVariable::Py(FrameId::World), iso.translation.y);
    initial_state.set_variable(&StateVariable::Pz(FrameId::World), iso.translation.z);

    let q_rot = iso.rotation.quaternion();
    initial_state.set_variable(&StateVariable::Qx(body.clone(), world.clone()), q_rot.i);
    initial_state.set_variable(&StateVariable::Qy(body.clone(), world.clone()), q_rot.j);
    initial_state.set_variable(&StateVariable::Qz(body.clone(), world.clone()), q_rot.k);
    initial_state.set_variable(&StateVariable::Qw(body, world), q_rot.w);

    let ekf = Box::new(ExtendedKalmanFilter::new(initial_state, q, dynamics));
    Ok(Box::new(GaussianEstimatorNode::new(
        ctx.instance_name,
        ekf,
        input_builder,
        ctx.aiding,
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::config::{
        AckermannProcessNoiseConfig, EkfConfig, EkfInitialStateConfig, IntegratedImuConfig,
        QuadcopterProcessNoiseConfig,
    };

    use helios_core::data::primitives::FrameHandle;

    fn ekf_config_with(dynamics: EkfDynamicsConfig) -> EstimatorConfig {
        EstimatorConfig::Ekf(EkfConfig {
            dynamics,
            aiding: vec![],
            initial_state: EkfInitialStateConfig::default(),
        })
    }

    fn config() -> EstimatorConfig {
        ekf_config_with(EkfDynamicsConfig::IntegratedImu(IntegratedImuConfig {
            gravity_enu: [0.0, 0.0, -9.81],
            accel_noise_stddev: 0.1,
            gyro_noise_stddev: 0.01,
            accel_bias_instability: 0.001,
            gyro_bias_instability: 0.0001,
            accel_channel: "imu/accel".to_string(),
            gyro_channel: "imu/gyro".to_string(),
        }))
    }

    fn context(instance_name: &str) -> GaussianEstimatorBuildContext {
        GaussianEstimatorBuildContext {
            agent_handle: FrameHandle(0),
            instance_name: instance_name.to_string(),
            aiding: vec![],
        }
    }

    // The node name is the config-map key, not the kind: two `Ekf` estimators
    // under distinct keys must yield distinct node identities.
    #[test]
    fn node_name_is_the_config_key_not_the_kind() {
        let registry = AutonomyRegistry::default();
        let primary = build_ekf(config(), context("primary"), &registry).unwrap();
        let backup = build_ekf(config(), context("backup"), &registry).unwrap();

        assert_eq!(primary.name(), "primary");
        assert_eq!(backup.name(), "backup");
    }

    // Only the IntegratedImu dynamics kind is wired; the other two must be
    // rejected with an error, never panic, so a misconfigured stack fails at
    // build time with a legible message instead of at runtime.
    #[test]
    fn unimplemented_dynamics_kinds_return_err() {
        let registry = AutonomyRegistry::default();

        let ackermann = ekf_config_with(EkfDynamicsConfig::AckermannOdometry(
            AckermannProcessNoiseConfig {
                velocity_stddev: 0.1,
                yaw_rate_stddev: 0.01,
            },
        ));
        let quadcopter = ekf_config_with(EkfDynamicsConfig::Quadcopter(
            QuadcopterProcessNoiseConfig {
                force_stddev: 0.1,
                torque_stddev: 0.01,
            },
        ));

        assert!(build_ekf(ackermann, context("a"), &registry).is_err());
        assert!(build_ekf(quadcopter, context("q"), &registry).is_err());
    }

    // A non-default initial pose + uncertainty must build cleanly: this drives
    // the `pos_var` / `ori_var` values through the schema's P₀ and the EKF's
    // `tangent_dim == Q.nrows()` assert without a dimension mismatch.
    #[test]
    fn custom_initial_state_builds() {
        let registry = AutonomyRegistry::default();

        let EstimatorConfig::Ekf(mut ekf) = config() else {
            unreachable!("config() is an Ekf");
        };
        ekf.initial_state.x = 5.0;
        ekf.initial_state.heading_deg = 90.0;
        ekf.initial_state.position_uncertainty_m = 3.0;
        ekf.initial_state.orientation_uncertainty_deg = 10.0;

        let node = build_ekf(EstimatorConfig::Ekf(ekf), context("custom"), &registry);
        assert!(node.is_ok());
    }
}
