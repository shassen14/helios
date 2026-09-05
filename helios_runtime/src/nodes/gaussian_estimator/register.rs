//! Registers Gaussian filter factories (EKF, UKF).

use super::input::IntegratedImuInputBuilder;
use super::node::GaussianEstimatorNode;

use crate::config::{EkfDynamicsConfig, EstimatorConfig};
use crate::nodes::gaussian_estimator::EstimatorInputBuilder;
use crate::pipeline::node::PipelineNode;
use crate::registry::{contexts::GaussianEstimatorBuildContext, AutonomyRegistry};

use helios_core::estimation::dynamics::integrated_imu::{
    ImuInitialUncertainty, ImuProcessNoise, IntegratedImuModel,
};
use helios_core::estimation::dynamics::EstimationDynamics;
use helios_core::estimation::filters::ekf::ExtendedKalmanFilter;
use helios_core::estimation::schema::check_measurement_state_agreement;
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
use helios_core::state::{Component, Quantity};

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::sync::Arc;

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
                    ImuProcessNoise {
                        accel_noise_var: c.accel_noise_stddev.powi(2),
                        gyro_noise_var: c.gyro_noise_stddev.powi(2),
                        accel_bias_var: c.accel_bias_instability.powi(2),
                        gyro_bias_var: c.gyro_bias_instability.powi(2),
                    },
                    ImuInitialUncertainty {
                        pos_var: init.position_uncertainty_m.powi(2),
                        vel_var: init.velocity_uncertainty_mps.powi(2),
                        ori_var: init.orientation_uncertainty_deg.to_radians().powi(2),
                        accel_bias_var: c.accel_bias_uncertainty_mps2.powi(2),
                        gyro_bias_var: c.gyro_bias_uncertainty_radps.powi(2),
                    },
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

    // The dynamics model is the single author of its *base* state shape:
    // process noise (Q) and initial covariance (P₀) are both baked into the
    // schema from the variances passed to `new` above, so we read them back
    // rather than rebuilding them here.
    //
    // Config-declared augmentations extend that base with per-sensor nuisance
    // blocks (e.g. magnetometer hard-iron bias) the dynamics knows nothing
    // about. Composing them here, before Q and P₀ are read, is what routes each
    // block's random-walk into the filter's process noise and its init
    // uncertainty into P₀. With no augmentations the base schema passes through
    // unchanged, so an un-augmented stack is byte-identical to before.
    let base_schema = dynamics.schema();
    let schema = if ctx.augmentation_blocks.is_empty() {
        base_schema
    } else {
        Arc::new(base_schema.extended(ctx.augmentation_blocks))
    };
    let q = schema.process_noise().clone();

    // Every aiding measurement must declare its axes the way the composed state
    // does — same convention for a directly-observed quantity, same convention
    // for any frame the state also anchors. A disagreement is a construction
    // error surfaced here, before the node is built, naming the estimator and
    // the offending sensor channel so a misconfigured stack fails legibly.
    for handler in &ctx.aiding {
        check_measurement_state_agreement(&schema, &handler.schema()).map_err(|e| {
            format!(
                "estimator '{}' aiding channel {}: {e}",
                ctx.instance_name,
                handler.channel()
            )
        })?;
    }

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
    let odom = FrameId::Odom(agent_handle);

    initial_state.set_variable(
        &StateVariable::new(Quantity::Position(odom.clone()), Component::X),
        iso.translation.x,
    );
    initial_state.set_variable(
        &StateVariable::new(Quantity::Position(odom.clone()), Component::Y),
        iso.translation.y,
    );
    initial_state.set_variable(
        &StateVariable::new(Quantity::Position(odom.clone()), Component::Z),
        iso.translation.z,
    );

    let q_rot = iso.rotation.quaternion();
    initial_state.set_variable(
        &StateVariable::new(
            Quantity::Orientation {
                from: body.clone(),
                to: odom.clone(),
            },
            Component::X,
        ),
        q_rot.i,
    );
    initial_state.set_variable(
        &StateVariable::new(
            Quantity::Orientation {
                from: body.clone(),
                to: odom.clone(),
            },
            Component::Y,
        ),
        q_rot.j,
    );
    initial_state.set_variable(
        &StateVariable::new(
            Quantity::Orientation {
                from: body.clone(),
                to: odom.clone(),
            },
            Component::Z,
        ),
        q_rot.k,
    );
    initial_state.set_variable(
        &StateVariable::new(
            Quantity::Orientation {
                from: body,
                to: odom,
            },
            Component::W,
        ),
        q_rot.w,
    );

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
    use crate::pipeline::node::TickContext;
    use crate::port::{ChannelKey, InternalChannel, PortBus, PortDescriptor};
    use crate::runtime::AgentRuntime;

    use crate::nodes::gaussian_estimator::{AidingHandler, TypedAidingHandler};
    use crate::port::SensorChannel;

    use helios_core::data::envelope::SensorReading;
    use helios_core::data::ports::TfProvider;
    use helios_core::data::primitives::FrameHandle;
    use helios_core::data::sensor::LinearAcceleration3D;
    use helios_core::data::MonotonicTime;
    use helios_core::estimation::augmentation::{augmentation_block, MAGNETOMETER_BIAS};
    use helios_core::estimation::measurement::MeasurementModel;
    use helios_core::estimation::schema::{
        MeasurementSchema, MeasurementSchemaBlock, StateSchemaBlock,
    };
    use helios_core::frames::transforms::{Convention, ErasedTransform};

    use nalgebra::{DMatrix, DVector};

    fn ekf_config_with(dynamics: EkfDynamicsConfig) -> EstimatorConfig {
        EstimatorConfig::Ekf(EkfConfig {
            dynamics,
            aiding: vec![],
            augmentation: vec![],
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
            accel_bias_uncertainty_mps2: 0.1,
            gyro_bias_uncertainty_radps: 0.01,
            accel_channel: "imu/accel".to_string(),
            gyro_channel: "imu/gyro".to_string(),
        }))
    }

    fn context(instance_name: &str) -> GaussianEstimatorBuildContext {
        context_with(instance_name, vec![])
    }

    fn context_with(
        instance_name: &str,
        augmentation_blocks: Vec<helios_core::estimation::schema::StateSchemaBlock>,
    ) -> GaussianEstimatorBuildContext {
        GaussianEstimatorBuildContext {
            agent_handle: FrameHandle(0),
            instance_name: instance_name.to_string(),
            aiding: vec![],
            augmentation_blocks,
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

    // --- Augmentation composition (R2) ---

    /// A runtime that never resolves a transform. Augmentation composition is
    /// observed on a cold-start tick (no aiding, predict skipped), so no TF
    /// lookup is exercised.
    struct MockRuntime;

    impl AgentRuntime for MockRuntime {
        fn get_transform(
            &self,
            _: FrameId,
            _: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            None
        }
        fn now(&self) -> MonotonicTime {
            MonotonicTime(0.0)
        }
    }

    /// Builds an EKF node from `config()` with the given augmentation blocks,
    /// runs one cold-start tick (empty predict-side channels, so predict is
    /// skipped), and returns the `FrameAwareState` the node publishes — its
    /// schema is the composed state we assert on.
    fn published_state(augmentation_blocks: Vec<StateSchemaBlock>) -> FrameAwareState {
        let registry = AutonomyRegistry::default();
        let node = build_ekf(
            config(),
            context_with("aug", augmentation_blocks),
            &registry,
        )
        .expect("augmented EKF must build");

        // A bus carrying the node's state output plus every channel it reads
        // (left empty → predict is skipped, cold start).
        let mut outputs: Vec<ChannelKey> = node.port_descriptor().required_inputs.clone();
        outputs.push(InternalChannel::of::<FrameAwareState>().into());
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs,
            rate: None,
        };
        let bus = PortBus::new(&[descriptor]);

        node.execute(
            &bus,
            &MockRuntime,
            TickContext {
                now: MonotonicTime(0.0),
                dt: 0.1,
                node_id: 0,
            },
        );

        bus.read::<FrameAwareState>(InternalChannel::of::<FrameAwareState>().into())
            .expect("node publishes its state")
            .value
            .clone()
    }

    // A declared augmentation must actually grow the running filter's state: the
    // dynamics-authored 16-state INS base gains a 3-DOF MagBias block, and that
    // block's variables land in the composed layout under the sensor frame the
    // block was tagged with.
    #[test]
    fn augmentation_blocks_extend_the_published_filter_state() {
        let sensor = FrameId::Sensor(FrameHandle(7));
        let block = augmentation_block(MAGNETOMETER_BIAS, sensor.clone(), 5.0, 0.01)
            .expect("well-formed mag-bias block");

        let state = published_state(vec![block]);

        assert_eq!(state.schema().storage_dim(), 19, "16 base + 3 bias");
        assert!(state
            .schema()
            .storage_offset_of(&StateVariable::new(Quantity::MagBias(sensor), Component::X))
            .is_some());
    }

    // The empty-augmentation path must be byte-identical to the un-augmented
    // filter: the base schema passes straight through, no bias slots appear.
    #[test]
    fn no_augmentation_leaves_the_base_state_unchanged() {
        let state = published_state(vec![]);

        assert_eq!(state.schema().storage_dim(), 16);
        assert!(state
            .schema()
            .storage_offset_of(&StateVariable::new(
                Quantity::MagBias(FrameId::Sensor(FrameHandle(7))),
                Component::X,
            ))
            .is_none());
    }

    // --- Measurement/state agreement is enforced at build ---

    // Two mock measurement models with fixed schemas, to drive the agreement
    // check without a real sensor. Both are 3-DOF; only the declared frame and
    // convention differ, which is all the check reads.

    /// Declares its measurement in `Odom(FrameHandle(0))` / ENU — exactly how the
    /// IntegratedImu base state anchors position, so it agrees.
    struct AgreeingModel;

    impl MeasurementModel for AgreeingModel {
        fn dim(&self) -> usize {
            3
        }
        fn schema(&self) -> MeasurementSchema {
            MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::Odom(FrameHandle(0))),
                Convention::Enu,
            )])
        }
        fn predict_measurement(
            &self,
            _: &FrameAwareState,
            _: Option<&dyn TfProvider>,
            _: MonotonicTime,
        ) -> Option<DVector<f64>> {
            Some(DVector::zeros(3))
        }
    }

    /// Declares its measurement in the world frame, which the base state never
    /// anchors and which is not a sensor frame — an unanchorable disagreement.
    struct UnanchorableModel;

    impl MeasurementModel for UnanchorableModel {
        fn dim(&self) -> usize {
            3
        }
        fn schema(&self) -> MeasurementSchema {
            MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::World),
                Convention::Enu,
            )])
        }
        fn predict_measurement(
            &self,
            _: &FrameAwareState,
            _: Option<&dyn TfProvider>,
            _: MonotonicTime,
        ) -> Option<DVector<f64>> {
            Some(DVector::zeros(3))
        }
    }

    fn aiding_with(model: Box<dyn MeasurementModel>) -> Box<dyn AidingHandler> {
        Box::new(TypedAidingHandler::<LinearAcceleration3D>::new(
            SensorChannel::of::<Vec<SensorReading<LinearAcceleration3D>>>(),
            model,
            DMatrix::identity(3, 3),
        ))
    }

    // An aiding measurement whose schema agrees with the composed state must not
    // block the build.
    #[test]
    fn build_ekf_accepts_an_agreeing_aiding_measurement() {
        let registry = AutonomyRegistry::default();
        let mut ctx = context("agrees");
        ctx.aiding.push(aiding_with(Box::new(AgreeingModel)));

        assert!(build_ekf(config(), ctx, &registry).is_ok());
    }

    // An aiding measurement in a frame the state can't anchor is a config error:
    // the build must fail rather than run a filter whose innovation references a
    // frame the state never expresses.
    #[test]
    fn build_ekf_rejects_a_disagreeing_aiding_measurement() {
        let registry = AutonomyRegistry::default();
        let mut ctx = context("disagrees");
        ctx.aiding.push(aiding_with(Box::new(UnanchorableModel)));

        assert!(build_ekf(config(), ctx, &registry).is_err());
    }
}
