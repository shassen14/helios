use crate::simulation::core::{
    components::GroundTruthState,
    transforms::{EnuBodyPose, EnuVector},
};
use crate::simulation::plugins::autonomy::AutonomyPipelineComponent;

use helios_core::data::{MonotonicTime, Twist};
use helios_runtime::channels::{oracle_pose_channel, oracle_twist_channel};
use helios_runtime::{Health, Stamped, HOST_PRODUCER_ID};

use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;
use nalgebra::Isometry3;

/// Mirrors Avian3D physics state into [`GroundTruthState`] each tick:
/// pose (world ENU), linear/angular velocity (world ENU), and derived
/// linear acceleration (world ENU, finite-difference). Runs in
/// [`SimulationSet::StateSync`] after physics has stepped.
pub fn ground_truth_sync_system(
    mut query: Query<(
        &GlobalTransform,
        &LinearVelocity,
        &AngularVelocity,
        &mut GroundTruthState,
    )>,
    time: Res<Time>,
) {
    // Guard the very first tick (and any pathological zero-dt frame): the
    // finite-difference acceleration below divides by dt. Skipping the
    // whole sync is fine — `last_linear_velocity` simply stays at its
    // default zero, and the next non-degenerate tick produces a valid
    // acceleration.
    let dt = time.delta_secs_f64();
    if dt < 1e-6 {
        return;
    }

    for (transform, lin_vel, ang_vel, mut ground_truth) in &mut query {
        ground_truth.pose = EnuBodyPose::from(transform).0;
        ground_truth.angular_velocity =
            EnuVector::from(Vec3::new(ang_vel.x, ang_vel.y, ang_vel.z)).0;

        let current_linear_velocity_enu =
            EnuVector::from(Vec3::new(lin_vel.x, lin_vel.y, lin_vel.z)).0;

        let linear_acceleration_enu =
            (current_linear_velocity_enu - ground_truth.last_linear_velocity) / dt;

        ground_truth.linear_velocity = current_linear_velocity_enu;
        ground_truth.linear_acceleration = linear_acceleration_enu;
        ground_truth.last_linear_velocity = current_linear_velocity_enu;
    }
}

/// Publishes ground-truth pose and twist onto each agent's autonomy bus on
/// the canonical [`oracle_pose_channel`] and [`oracle_twist_channel`]
/// slots. Runs in [`SimulationSet::StateSync`] after
/// [`ground_truth_sync_system`].
///
/// Pose is published in world ENU (matching [`GroundTruthState`]). Twist
/// is converted into body FLU here — `linear` and `angular` are rotated
/// by `pose.rotation.inverse()` so mocks reading this channel see the
/// same body-frame convention real hardware delivers.
///
/// The bus timestamp source is `Time::elapsed_secs_f64`, identical to
/// what `SimRuntime::now()` returns later in the autonomy tick — so
/// `read_fresh(max_age)` comparisons across systems work without skew.
///
/// `bus.write` returns `Err(UnknownChannel)` if no node in the graph
/// consumes the slot; that is the documented "unused channel" outcome,
/// not an error worth surfacing, so the result is intentionally
/// discarded.
pub fn publish_oracle_channels_system(
    query: Query<(&GroundTruthState, &AutonomyPipelineComponent)>,
    time: Res<Time>,
) {
    let now = MonotonicTime(time.elapsed_secs_f64());

    for (ground_truth, autonomy_pipeline) in query {
        let pose = ground_truth.pose;
        let rot_inv = pose.rotation.inverse();

        // World ENU → body FLU. Documented on `oracle_twist_channel`.
        let linear_body = rot_inv * ground_truth.linear_velocity;
        let angular_body = rot_inv * ground_truth.angular_velocity;

        let stamped_pose: Stamped<Isometry3<f64>> = Stamped {
            value: pose,
            timestamp: now,
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        let stamped_twist = Stamped {
            value: Twist {
                linear: linear_body,
                angular: angular_body,
            },
            timestamp: now,
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        let bus = autonomy_pipeline.0.bus();
        let _ = bus.write(oracle_pose_channel(), stamped_pose);
        let _ = bus.write(oracle_twist_channel(), stamped_twist);
    }
}
