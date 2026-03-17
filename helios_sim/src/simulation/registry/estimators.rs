// helios_sim/src/simulation/registry/estimators.rs
//
// Registers all known state estimators with the AutonomyRegistry.
// To add a new estimator:
//   1. Implement StateEstimator in helios_core.
//   2. Add a register_estimator call below.
//   Zero spawning systems change.
//
// The EKF factory uses ctx.dynamics_factories to construct the dynamics model by key,
// so adding a new dynamics variant never requires touching this file either.

use bevy::prelude::*;
use helios_core::{
    estimation::{filters::ekf::ExtendedKalmanFilter, StateEstimator},
    frames::{
        layout::{standard_ins_state_layout, STANDARD_INS_STATE_DIM},
        FrameId, StateVariable,
    },
    models::estimation::measurement::imu::Imu6DofModel,
};
use nalgebra::DMatrix;

use crate::simulation::config::structs::{EkfDynamicsConfig, EstimatorConfig};

use super::{AutonomyRegistry, DynamicsBuildContext, EstimatorBuildContext};

pub struct DefaultEstimatorsPlugin;

impl Plugin for DefaultEstimatorsPlugin {
    fn build(&self, app: &mut App) {
        app.world_mut()
            .resource_mut::<AutonomyRegistry>()
            .register_estimator("Ekf", build_ekf)
            .register_estimator("Ukf", build_ukf);
    }
}

fn build_ekf(ctx: EstimatorBuildContext) -> Result<Box<dyn StateEstimator>, String> {
    let EstimatorConfig::Ekf(ekf_config) = ctx.estimator_cfg else {
        return Err("build_ekf received non-Ekf config".to_string());
    };

    let state_dim = STANDARD_INS_STATE_DIM;
    let agent_entity = ctx.agent_entity;

    // --- 1. Build the Q process-noise matrix from the dynamics config ---
    let mut q_matrix = DMatrix::<f64>::zeros(state_dim, state_dim);
    let dynamics_key = ekf_config.dynamics.get_kind_str();

    match &ekf_config.dynamics {
        EkfDynamicsConfig::IntegratedImu(noise_conf) => {
            let an_var = noise_conf.accel_noise_stddev.powi(2);
            let gn_var = noise_conf.gyro_noise_stddev.powi(2);
            let ab_var = noise_conf.accel_bias_instability.powi(2);
            let gb_var = noise_conf.gyro_bias_instability.powi(2);
            // Velocity noise
            q_matrix[(3, 3)] = an_var;
            q_matrix[(4, 4)] = an_var;
            q_matrix[(5, 5)] = an_var;
            // Orientation noise
            q_matrix[(6, 6)] = gn_var;
            q_matrix[(7, 7)] = gn_var;
            q_matrix[(8, 8)] = gn_var;
            // Accel bias noise
            q_matrix[(10, 10)] = ab_var;
            q_matrix[(11, 11)] = ab_var;
            q_matrix[(12, 12)] = ab_var;
            // Gyro bias noise
            q_matrix[(13, 13)] = gb_var;
            q_matrix[(14, 14)] = gb_var;
            q_matrix[(15, 15)] = gb_var;
        }
        EkfDynamicsConfig::AckermannOdometry(noise_conf) => {
            let v_var = noise_conf.velocity_stddev.powi(2);
            let w_var = noise_conf.yaw_rate_stddev.powi(2);
            // Minimal Q for a simple kinematic model — expand when implemented
            q_matrix[(3, 3)] = v_var;
            q_matrix[(5, 5)] = w_var;
        }
        EkfDynamicsConfig::Quadcopter(noise_conf) => {
            let f_var = noise_conf.force_stddev.powi(2);
            let t_var = noise_conf.torque_stddev.powi(2);
            q_matrix[(3, 3)] = f_var;
            q_matrix[(4, 4)] = f_var;
            q_matrix[(5, 5)] = f_var;
            q_matrix[(6, 6)] = t_var;
            q_matrix[(7, 7)] = t_var;
            q_matrix[(8, 8)] = t_var;
        }
    }

    // --- 2. Build the dynamics model via the dynamics registry snapshot ---
    let starting_pose = ctx.agent_config.starting_pose;
    let dynamics_ctx = DynamicsBuildContext {
        agent_entity,
        agent_config: ctx.agent_config,
        gravity_magnitude: ctx.gravity_magnitude,
    };

    let dynamics = ctx
        .dynamics_factories
        .get(dynamics_key)
        .ok_or_else(|| {
            format!(
                "No dynamics factory registered for '{dynamics_key}'. \
                 Register one via AutonomyRegistry::register_dynamics()."
            )
        })
        .and_then(|f| f(dynamics_ctx))?;

    // --- 3. Filter measurement models ---
    // IntegratedImu dynamics uses IMU data as control input (u), not as an aiding measurement.
    // Other dynamics use all sensor models for updates.
    let measurement_models = match &ekf_config.dynamics {
        EkfDynamicsConfig::IntegratedImu(_) => ctx
            .measurement_models
            .into_iter()
            .filter(|(_, m)| !m.as_any().is::<Imu6DofModel>())
            .collect(),
        _ => ctx.measurement_models,
    };

    // --- 4. Assemble and return the filter ---
    let agent_handle = helios_core::types::FrameHandle::from_entity(agent_entity);
    let state_layout = standard_ins_state_layout(agent_handle);
    let mut initial_state = helios_core::frames::FrameAwareState::new(state_layout, 1.0, 0.0);

    // Seed the EKF from the agent's known starting pose so the heading is correct
    // from tick 0. Without this, the EKF starts at identity (yaw=0, facing East)
    // regardless of the scenario starting_pose, causing a permanent heading offset
    // that GPS+IMU alone cannot correct (no magnetometer = no absolute heading source).
    let starting_iso = starting_pose.to_isometry();
    let body_frame = FrameId::Body(agent_handle);
    let world_frame = FrameId::World;
    initial_state.set_variable(
        &StateVariable::Px(FrameId::World),
        starting_iso.translation.x,
    );
    initial_state.set_variable(
        &StateVariable::Py(FrameId::World),
        starting_iso.translation.y,
    );
    initial_state.set_variable(
        &StateVariable::Pz(FrameId::World),
        starting_iso.translation.z,
    );
    let q = starting_iso.rotation.quaternion();
    initial_state.set_variable(
        &StateVariable::Qx(body_frame.clone(), world_frame.clone()),
        q.i,
    );
    initial_state.set_variable(
        &StateVariable::Qy(body_frame.clone(), world_frame.clone()),
        q.j,
    );
    initial_state.set_variable(
        &StateVariable::Qz(body_frame.clone(), world_frame.clone()),
        q.k,
    );
    initial_state.set_variable(&StateVariable::Qw(body_frame, world_frame), q.w);

    info!(
        "AutonomyRegistry: built EKF with '{}' dynamics for agent {:?}.",
        dynamics_key, agent_entity
    );

    Ok(Box::new(ExtendedKalmanFilter::new(
        initial_state,
        q_matrix,
        dynamics,
        measurement_models,
    )))
}

fn build_ukf(_ctx: EstimatorBuildContext) -> Result<Box<dyn StateEstimator>, String> {
    Err("UKF not yet implemented".to_string())
}
