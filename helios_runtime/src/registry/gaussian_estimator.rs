//! Registers Gaussian filter factories (EKF, UKF).

use helios_core::estimation::filters::ekf::ExtendedKalmanFilter;
use helios_core::frames::layout::{standard_ins_state_layout, STANDARD_INS_STATE_DIM};
use helios_core::frames::{FrameId, StateVariable};
use nalgebra::DMatrix;

use crate::config::{EkfDynamicsConfig, EstimatorConfig};
use crate::pipeline::builders::estimator::IntegratedImuInputBuilder;
use crate::pipeline::nodes::gaussian_estimator::GaussianEstimatorNode;
use crate::pipeline::node::PipelineNode;

use super::{
    contexts::{DynamicsBuildContext, GaussianEstimatorBuildContext},
    AutonomyRegistry,
};

pub fn register(registry: &mut AutonomyRegistry) {
    registry.register_gaussian_estimator("Ekf", build_ekf);
    registry.register_gaussian_estimator("Ukf", |_, _, _| {
        Err("UKF not yet implemented".to_string())
    });
}

fn build_ekf(
    config: EstimatorConfig,
    ctx: GaussianEstimatorBuildContext,
    registry: &AutonomyRegistry,
) -> Result<Box<dyn PipelineNode>, String> {
    let EstimatorConfig::Ekf(ekf_config) = config else {
        return Err("build_ekf received non-Ekf config".to_string());
    };

    let agent_handle = ctx.agent_handle;
    let dim = STANDARD_INS_STATE_DIM;

    // --- 1. Build Q process-noise matrix ---
    let mut q = DMatrix::<f64>::zeros(dim, dim);
    let dynamics_key = ekf_config.dynamics.get_kind_str();

    match &ekf_config.dynamics {
        EkfDynamicsConfig::IntegratedImu(n) => {
            let an = n.accel_noise_stddev.powi(2);
            let gn = n.gyro_noise_stddev.powi(2);
            let ab = n.accel_bias_instability.powi(2);
            let gb = n.gyro_bias_instability.powi(2);
            // indices per standard_ins_state_layout: Vx/Vy/Vz = 3-5, Qx-Qw = 6-9, Ax-Az = 10-12, Wx-Wz = 13-15
            q[(3, 3)] = an; q[(4, 4)] = an; q[(5, 5)] = an;
            q[(6, 6)] = gn; q[(7, 7)] = gn; q[(8, 8)] = gn; q[(9, 9)] = gn;
            q[(10, 10)] = ab; q[(11, 11)] = ab; q[(12, 12)] = ab;
            q[(13, 13)] = gb; q[(14, 14)] = gb; q[(15, 15)] = gb;
        }
        EkfDynamicsConfig::AckermannOdometry(n) => {
            let v = n.velocity_stddev.powi(2);
            let w = n.yaw_rate_stddev.powi(2);
            q[(3, 3)] = v; q[(5, 5)] = w;
        }
        EkfDynamicsConfig::Quadcopter(n) => {
            let f = n.force_stddev.powi(2);
            let t = n.torque_stddev.powi(2);
            q[(3, 3)] = f; q[(4, 4)] = f; q[(5, 5)] = f;
            q[(6, 6)] = t; q[(7, 7)] = t; q[(8, 8)] = t; q[(9, 9)] = t;
        }
    }

    // --- 2. Build dynamics via registry ---
    let dynamics = registry.build_dynamics(
        dynamics_key,
        DynamicsBuildContext {
            agent_handle,
            gravity: ctx.gravity,
        },
    )?;

    // --- 3. Seed initial state from starting pose ---
    let state_layout = standard_ins_state_layout(agent_handle);
    let mut initial_state =
        helios_core::frames::FrameAwareState::new(state_layout, 1.0, 0.0);

    let iso = ctx.starting_pose;
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

    // Starting orientation is known exactly — tighten quaternion covariance to avoid
    // spurious position-orientation cross-covariance corrections from early GPS updates.
    let q_var: f64 = 1e-4;
    for (i, var) in initial_state.state.layout.iter().enumerate() {
        if matches!(
            var,
            StateVariable::Qx(_, _)
                | StateVariable::Qy(_, _)
                | StateVariable::Qz(_, _)
                | StateVariable::Qw(_, _)
        ) {
            initial_state.covariance[(i, i)] = q_var;
        }
    }

    // --- 4. Assemble node ---
    let ekf = Box::new(ExtendedKalmanFilter::new(initial_state, q, dynamics));
    let input_builder = Box::new(IntegratedImuInputBuilder::new());
    Ok(Box::new(GaussianEstimatorNode::new(
        "ekf",
        ekf,
        input_builder,
        ctx.aiding,
    )))
}
