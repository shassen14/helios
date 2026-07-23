use crate::brain_bridge::AutonomyPipelineComponent;
use crate::core::{
    components::GroundTruthState,
    transforms::{EnuBodyPose, EnuVector},
};

use helios_core::data::{MonotonicTime, Twist};
use helios_runtime::channels::{oracle_pose_channel, oracle_twist_channel};
use helios_runtime::{Health, Stamped, HOST_PRODUCER_ID};

use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;
use nalgebra::Isometry3;

/// Mirrors Avian3D physics state into [`GroundTruthState`] each tick:
/// pose (world ENU), linear/angular velocity (world ENU), and derived
/// linear acceleration (world ENU, finite-difference). Runs in
/// [`SimulationSet::StateSync`](crate::core::app_state::SimulationSet::StateSync) after physics has stepped.
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

        let current_angular_velocity_enu =
            EnuVector::from(Vec3::new(ang_vel.x, ang_vel.y, ang_vel.z)).0;

        let angular_acceleration_enu =
            (current_angular_velocity_enu - ground_truth.last_angular_velocity) / dt;

        let current_linear_velocity_enu =
            EnuVector::from(Vec3::new(lin_vel.x, lin_vel.y, lin_vel.z)).0;

        let linear_acceleration_enu =
            (current_linear_velocity_enu - ground_truth.last_linear_velocity) / dt;

        ground_truth.linear_velocity = current_linear_velocity_enu;
        ground_truth.angular_velocity = current_angular_velocity_enu;
        ground_truth.linear_acceleration = linear_acceleration_enu;
        ground_truth.angular_acceleration = angular_acceleration_enu;
        ground_truth.last_linear_velocity = current_linear_velocity_enu;
        ground_truth.last_angular_velocity = current_angular_velocity_enu;
    }
}

/// Publishes ground-truth pose and twist onto each agent's autonomy bus on
/// the canonical [`oracle_pose_channel`] and [`oracle_twist_channel`]
/// slots. Runs in [`SimulationSet::StateSync`](crate::core::app_state::SimulationSet::StateSync) after
/// [`ground_truth_sync_system`].
///
/// Pose and twist are both published in world ENU (matching
/// [`GroundTruthState`]). Aligns with `STANDARD_INS_LAYOUT`'s velocity
/// convention so the mock-oracle estimator can passthrough into a
/// `FrameAwareState` without rotating.
///
/// The bus timestamp source is `Time::elapsed_secs_f64`, identical to
/// what `SimRuntime::now()` returns later in the autonomy tick — so
/// `read_fresh(max_age)` comparisons across systems work without skew.
///
/// `bus.write` returns `Err(UnknownChannel)` if no node in the graph
/// consumes the slot. For the oracle that is expected, not a bug: nothing
/// reads `oracle/pose` or `oracle/twist` until a consumer — a mock-oracle
/// estimator or a visualization layer — is wired into the pipeline. So instead
/// of the crate-wide "warn loudly" rule for a dropped write, this system traces
/// the miss once (naming the channel), then stays quiet — a real wiring bug
/// elsewhere still screams; this known no-consumer case says its piece once and
/// does not spam the fixed-rate tick.
pub fn publish_oracle_channels_system(
    query: Query<(&GroundTruthState, &AutonomyPipelineComponent)>,
    time: Res<Time>,
    mut has_logged_first_tick: Local<bool>,
    mut has_warned_no_consumer: Local<bool>,
) {
    let now = MonotonicTime(time.elapsed_secs_f64());

    // One-shot verification trace: fires exactly once the first time this
    // system actually executes. Used to confirm the schedule wiring is
    // correct (system is in StateSync, run_if(Running) is satisfied) — a
    // silent run cannot otherwise be observed from outside, because the
    // bus writes below land on slots that don't exist yet until a
    // consumer is built. Cheap to leave in; remove if it ever becomes
    // noise. Enable with RUST_LOG=helios_sim::oracle=trace.
    if !*has_logged_first_tick {
        tracing::trace!(
            target: "helios_sim::oracle",
            agent_count = query.iter().len(),
            "publish_oracle_channels_system: first tick"
        );
        *has_logged_first_tick = true;
    }

    for (ground_truth, autonomy_pipeline) in query {
        let pose = ground_truth.pose;

        let stamped_pose: Stamped<Isometry3<f64>> = Stamped {
            value: pose,
            timestamp: now,
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        // Twist is published in the world frame, matching the velocity
        // convention a passthrough estimator expects, so a mock oracle can copy
        // it straight into a FrameAwareState without a frame rotation.
        let stamped_twist = Stamped {
            value: Twist {
                linear: ground_truth.linear_velocity,
                angular: ground_truth.angular_velocity,
            },
            timestamp: now,
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        let bus = autonomy_pipeline.0.bus();
        let pose_result = bus.write(oracle_pose_channel(), stamped_pose);
        let twist_result = bus.write(oracle_twist_channel(), stamped_twist);

        // TODO: should check for actual error if we have more variants
        if pose_result.is_err() && !*has_warned_no_consumer {
            tracing::trace!(
                target: "helios_sim::oracle",
                channel = oracle_pose_channel().to_string(),
                "oracle channel has no consumer; write dropped"
            );
            *has_warned_no_consumer = true;
        }

        if twist_result.is_err() && !*has_warned_no_consumer {
            tracing::trace!(
                target: "helios_sim::oracle",
                channel = oracle_twist_channel().to_string(),
                "oracle channel has no consumer; write dropped"
            );
            *has_warned_no_consumer = true;
        }
    }
}
