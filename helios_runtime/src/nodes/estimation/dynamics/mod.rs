//! The dynamics component: a process model, the input builder that feeds its
//! predict, and how a pose prior is written into its state.
//!
//! - `input` — [`EstimatorInputBuilder`], which assembles the control vector.
//! - `integrated_imu` — the built-in `IntegratedImu` kind.

mod input;
mod integrated_imu;

pub use input::EstimatorInputBuilder;
pub(crate) use integrated_imu::{
    IntegratedImuInputBuilder, DEFAULT_ACCEL_BIAS_UNCERTAINTY_MPS2,
    DEFAULT_GYRO_BIAS_UNCERTAINTY_RADPS, DEFAULT_ORIENTATION_UNCERTAINTY_DEG,
    DEFAULT_POSITION_UNCERTAINTY_M, DEFAULT_VELOCITY_UNCERTAINTY_MPS, INTEGRATED_IMU_KIND,
};

use crate::nodes::estimation::EstimatorComponents;

use helios_core::estimation::dynamics::EstimationDynamics;
use helios_core::estimation::schema::{check_input_agreement, InputAgreementError};
use helios_core::prelude::AgentId;
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::transforms::Transform;
use helios_core::spatial::{FrameAwareState, FrameId};

/// Writes a body pose prior (`base_link` in `odom`) into a freshly seeded
/// state, or says why this dynamics' state can't hold it.
pub type SeedPose = fn(&mut FrameAwareState, &AgentId, Transform<Flu, Enu>) -> Result<(), String>;

/// What a dynamics factory returns: the process model, the input builder that
/// feeds it, and how a pose prior maps into its state.
///
/// The model and builder are checked against each other on construction, so
/// a pair that disagrees on the input layout can't be built.
pub struct DynamicsComponent {
    dynamics: Box<dyn EstimationDynamics>,
    input: Box<dyn EstimatorInputBuilder>,
    seed_pose: SeedPose,
}

impl DynamicsComponent {
    /// Pairs `dynamics` with the builder that feeds it, seeding a pose prior
    /// as a full 3D pose of `base_link` in `odom`.
    ///
    /// Fails if the builder's declared input differs, block by block, from the
    /// input the dynamics consume: same length is not enough, because an
    /// accelerometer/gyro swap is the same length.
    pub fn new(
        dynamics: Box<dyn EstimationDynamics>,
        input: Box<dyn EstimatorInputBuilder>,
    ) -> Result<Self, InputAgreementError> {
        check_input_agreement(&dynamics.input_schema(), &input.input_schema())?;
        Ok(Self {
            dynamics,
            input,
            seed_pose: seed_body_pose_in_odom,
        })
    }

    /// Replaces how a pose prior is written, for a state that holds a pose
    /// some other way (a planar model storing a heading angle, say).
    pub fn with_pose_seeding(mut self, seed_pose: SeedPose) -> Self {
        self.seed_pose = seed_pose;
        self
    }

    /// The process model.
    pub(crate) fn dynamics(&self) -> &dyn EstimationDynamics {
        &*self.dynamics
    }

    /// Writes `pose` into `state` the way this dynamics' state holds a pose.
    pub(crate) fn seed_pose(
        &self,
        state: &mut FrameAwareState,
        agent: &AgentId,
        pose: Transform<Flu, Enu>,
    ) -> Result<(), String> {
        (self.seed_pose)(state, agent, pose)
    }

    /// Takes the model and the builder out, for the filter and the node.
    pub(crate) fn into_parts(
        self,
    ) -> (Box<dyn EstimationDynamics>, Box<dyn EstimatorInputBuilder>) {
        (self.dynamics, self.input)
    }
}

/// The default [`SeedPose`]: the state's `base_link`-in-`odom` position and
/// orientation blocks, written whole or not at all.
fn seed_body_pose_in_odom(
    state: &mut FrameAwareState,
    agent: &AgentId,
    pose: Transform<Flu, Enu>,
) -> Result<(), String> {
    state
        .set_pose(
            FrameId::base_link(agent.clone()),
            FrameId::odom(agent.clone()),
            pose,
        )
        .map_err(|e| e.to_string())
}

pub(super) fn register(components: &mut EstimatorComponents) {
    integrated_imu::register(components);
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::{AutonomyRegistry, BuildContext, ComponentError, NoParams};
    use crate::port::{ChannelKey, PortBus};
    use crate::prelude::TickContext;

    use helios_core::estimation::dynamics::integrated_imu::{
        ImuInitialUncertainty, ImuProcessNoise, IntegratedImuModel,
    };
    use helios_core::estimation::schema::{InputSchema, InputSchemaBlock, StateSchema};
    use helios_core::estimation::EstimatorInputs;
    use helios_core::kernel::integrators::Integrator;
    use helios_core::prelude::MonotonicTime;
    use helios_core::spatial::state::{Component, Quantity};
    use helios_core::spatial::transforms::Convention;
    use helios_core::spatial::StateVariable;

    use nalgebra::{DVector, Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::collections::HashSet;
    use std::sync::Arc;

    const NODE: &str = "primary";
    const DUMMY_KIND: &str = "Stationary";

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn body() -> FrameId {
        FrameId::base_link(agent())
    }

    /// The input a dummy model consumes: one body angular velocity.
    fn rate_input() -> Arc<InputSchema> {
        Arc::new(InputSchema::compose(vec![InputSchemaBlock::new(
            Quantity::AngularVelocity(body()),
            Convention::Flu,
        )]))
    }

    /// A dummy process model: a stationary body pose that never moves.
    #[derive(Debug)]
    struct Stationary {
        schema: Arc<StateSchema>,
    }

    impl Stationary {
        /// Borrows the INS state layout, so the state holds a body pose.
        fn new() -> Self {
            let ins = IntegratedImuModel::new(
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
            Self {
                schema: ins.schema(),
            }
        }
    }

    impl EstimationDynamics for Stationary {
        fn input_schema(&self) -> Arc<InputSchema> {
            rate_input()
        }

        fn schema(&self) -> Arc<StateSchema> {
            Arc::clone(&self.schema)
        }

        fn propagate(
            &self,
            x: &DVector<f64>,
            _u: &DVector<f64>,
            _t: f64,
            _dt: f64,
            _integrator: &dyn Integrator<f64>,
        ) -> DVector<f64> {
            x.clone()
        }
    }

    /// A dummy input builder declaring whatever input it was given.
    struct Declares(Arc<InputSchema>);

    impl EstimatorInputBuilder for Declares {
        fn input_schema(&self) -> Arc<InputSchema> {
            Arc::clone(&self.0)
        }

        fn assemble(&self, _bus: &PortBus, _tick: &TickContext) -> Option<EstimatorInputs> {
            None
        }

        fn required_channels(&self) -> &[ChannelKey] {
            &[]
        }

        fn optional_channels(&self) -> &[ChannelKey] {
            &[]
        }
    }

    /// The same length as [`rate_input`], but a different quantity.
    fn force_input() -> Arc<InputSchema> {
        Arc::new(InputSchema::compose(vec![InputSchemaBlock::new(
            Quantity::SpecificForce(body()),
            Convention::Flu,
        )]))
    }

    fn build_stationary(_: NoParams, _: &BuildContext<'_>) -> Result<DynamicsComponent, String> {
        DynamicsComponent::new(
            Box::new(Stationary::new()),
            Box::new(Declares(rate_input())),
        )
        .map_err(|e| e.to_string())
    }

    fn build_mismatched(_: NoParams, _: &BuildContext<'_>) -> Result<DynamicsComponent, String> {
        DynamicsComponent::new(
            Box::new(Stationary::new()),
            Box::new(Declares(force_input())),
        )
        .map_err(|e| e.to_string())
    }

    fn build(
        registry: &AutonomyRegistry,
        section: &str,
    ) -> Result<DynamicsComponent, ComponentError> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(agent(), NODE, &channels);
        registry
            .extension::<EstimatorComponents>()
            .expect("the default registry has estimator components")
            .build_dynamics("dynamics", section, &ctx)
            .map(|built| built.component)
    }

    /// A dynamics kind registered from outside the built-ins builds through the
    /// registry.
    #[test]
    fn a_registered_dynamics_kind_builds() {
        let mut registry = AutonomyRegistry::default();
        registry
            .extension_mut::<EstimatorComponents>()
            .register_dynamics(DUMMY_KIND, build_stationary)
            .expect("not a built-in kind");

        let Ok(component) = build(&registry, &format!("kind = \"{DUMMY_KIND}\"")) else {
            panic!("the dummy dynamics builds");
        };
        assert_eq!(
            component.dynamics().input_schema().blocks(),
            rate_input().blocks()
        );
    }

    #[test]
    fn registering_a_built_in_dynamics_kind_again_is_rejected() {
        let mut registry = AutonomyRegistry::default();
        let err = registry
            .extension_mut::<EstimatorComponents>()
            .register_dynamics(INTEGRATED_IMU_KIND, build_stationary)
            .expect_err("IntegratedImu is a built-in");
        assert_eq!(
            err.to_string(),
            format!("dynamics kind '{INTEGRATED_IMU_KIND}' is already registered")
        );
    }

    /// A builder supplying a different quantity in the same number of rows is
    /// refused: the swap a length check would let through.
    #[test]
    fn a_builder_disagreeing_with_its_dynamics_is_refused() {
        let mut registry = AutonomyRegistry::default();
        registry
            .extension_mut::<EstimatorComponents>()
            .register_dynamics(DUMMY_KIND, build_mismatched)
            .expect("not a built-in kind");

        let Err(err) = build(&registry, &format!("kind = \"{DUMMY_KIND}\"")) else {
            panic!("a disagreeing pair must not build");
        };
        assert!(matches!(err, ComponentError::BuildFailed { .. }), "{err}");
        assert_eq!(
            force_input().dim(),
            rate_input().dim(),
            "same length, so only the block check catches it"
        );
    }

    fn pose() -> Transform<Flu, Enu> {
        Transform::from_isometry(Isometry3::from_parts(
            Translation3::new(4.0, -2.0, 0.5),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 1.2),
        ))
    }

    /// The default seeding writes the pose into the `base_link`-in-`odom`
    /// blocks.
    #[test]
    fn default_seeding_writes_the_body_pose_in_odom() {
        let component = DynamicsComponent::new(
            Box::new(Stationary::new()),
            Box::new(Declares(rate_input())),
        )
        .expect("agreeing pair");
        let mut state =
            FrameAwareState::from_schema(component.dynamics().schema(), MonotonicTime(0.0));

        component
            .seed_pose(&mut state, &agent(), pose())
            .expect("the INS schema holds a body pose");

        let east = state
            .schema()
            .storage_offset_of(&StateVariable::new(
                Quantity::Position(FrameId::odom(agent())),
                Component::X,
            ))
            .expect("position block");
        assert_eq!(state.mean[east], 4.0);
        let seeded = state
            .pose::<Flu, Enu>(body(), FrameId::odom(agent()))
            .expect("pose reads back");
        assert!((seeded.into_inner().rotation.angle() - 1.2).abs() < 1e-12);
    }

    /// A component can replace the seeding, for a state that holds a pose
    /// another way; the override is what runs.
    #[test]
    fn a_seeding_override_replaces_the_default() {
        fn refuse(
            _: &mut FrameAwareState,
            _: &AgentId,
            _: Transform<Flu, Enu>,
        ) -> Result<(), String> {
            Err("planar state: no 3D pose".to_string())
        }

        let component = DynamicsComponent::new(
            Box::new(Stationary::new()),
            Box::new(Declares(rate_input())),
        )
        .expect("agreeing pair")
        .with_pose_seeding(refuse);
        let mut state =
            FrameAwareState::from_schema(component.dynamics().schema(), MonotonicTime(0.0));

        assert_eq!(
            component.seed_pose(&mut state, &agent(), pose()),
            Err("planar state: no 3D pose".to_string())
        );
    }
}
