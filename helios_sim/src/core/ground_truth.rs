use crate::brain_bridge::{AgentIdComponent, AutonomyPipelineComponent};
use crate::core::components::GroundTruthState;
use crate::core::run_clock::RunClock;
use crate::core::transforms::{
    bevy_transform_to_transform_bevy, vec3_to_freevector_bevy, FromBevy, TfTree,
};

use helios_core::interchange::motion::StampedTwist;
use helios_core::interchange::perception::contact::Contacts;
use helios_core::prelude::{AgentId, MonotonicTime};
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::quantities::FreeVector;
use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};
use helios_core::spatial::transforms::{
    Convention, ConventionOf, ErasedTransform, Transform as CoreTransform,
};
use helios_core::spatial::FrameId;
use helios_runtime::channels::{truth_contacts, truth_tf, truth_twist};
use helios_runtime::port::{ChannelError, ChannelKey};
use helios_runtime::{BodyChannel, Health, Provenance, Stamped, HOST_PRODUCER_ID};

use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;
use std::collections::HashSet;

/// The unit that publishes ground truth: the simulator's truth rig, a perfect
/// sensor every simulated body carries.
pub const TRUTH_RIG_UNIT: &str = "truth_rig";

/// The datum's axes: the world axes [`GroundTruthState`] holds, which truth is
/// published in.
type DatumAxes = Enu;

/// The axis convention of the world's datum, as the host declares it.
pub const DATUM_CONVENTION: Convention = DatumAxes::CONVENTION;

/// Mirrors Avian3D physics state into [`GroundTruthState`] each tick, at
/// the step's instant: pose (world ENU), linear/angular velocity (world
/// ENU), and derived accelerations (world ENU, finite-difference, the mean
/// over the step). Runs in
/// [`SimulationSet::StateSync`](crate::core::app_state::SimulationSet::StateSync),
/// after physics has stepped and transforms are propagated, only while
/// `Running`: physics is paused before, so the spawn state stands.
pub fn ground_truth_sync_system(
    mut query: Query<(
        &GlobalTransform,
        &LinearVelocity,
        &AngularVelocity,
        &mut GroundTruthState,
    )>,
    clock: RunClock,
    fixed: Res<Time<Fixed>>,
) {
    // Runs only while `Running`, when the clock has started.
    let Some(now) = clock.now() else {
        return;
    };
    // The step the velocities differ over, never zero.
    let dt = fixed.timestep().as_secs_f64();

    for (transform, lin_vel, ang_vel, mut ground_truth) in &mut query {
        ground_truth.at = now;

        let world: CoreTransform<Flu, Enu> =
            bevy_transform_to_transform_bevy(transform.compute_transform()).from_bevy();
        ground_truth.pose = world.into_inner();

        let angular_velocity_enu: FreeVector<Enu> =
            vec3_to_freevector_bevy(Vec3::new(ang_vel.x, ang_vel.y, ang_vel.z)).from_bevy();
        let current_angular_velocity_enu = angular_velocity_enu.into_inner();

        let angular_acceleration_enu =
            (current_angular_velocity_enu - ground_truth.last_angular_velocity) / dt;

        let linear_velocity_enu: FreeVector<Enu> =
            vec3_to_freevector_bevy(Vec3::new(lin_vel.x, lin_vel.y, lin_vel.z)).from_bevy();
        let current_linear_velocity_enu = linear_velocity_enu.into_inner();

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

/// The channels the truth rig writes for `agent`, as the body publishes
/// them: `truth.tf` and `truth.twist` ([`publish_truth_channels_system`])
/// and `truth.contacts`
/// ([`publish_truth_contacts_system`](crate::core::truth_contacts::publish_truth_contacts_system)),
/// each with one instance, the agent's `base_link` (its pose expressed in
/// the world's `datum`, its twist relative to it, what it touched), exact
/// because the simulator is ground truth, and written every tick, at the
/// host's `tick_rate_hz`.
pub fn truth_body_channels(
    agent: &AgentId,
    datum: &FrameId,
    tick_rate_hz: f64,
) -> [BodyChannel; 3] {
    let base_link = FrameId::base_link(agent.clone());
    [
        BodyChannel::new(truth_tf(), TRUTH_RIG_UNIT, Provenance::Exact)
            .with_instances::<StampedTransform>([FrameEdge {
                child: base_link.clone(),
                parent: datum.clone(),
            }])
            .with_rate(tick_rate_hz),
        BodyChannel::new(truth_twist(), TRUTH_RIG_UNIT, Provenance::Exact)
            .with_instances::<StampedTwist<Flu>>([base_link.clone()])
            .with_reference(datum.clone())
            .with_rate(tick_rate_hz),
        BodyChannel::new(truth_contacts(), TRUTH_RIG_UNIT, Provenance::Exact)
            .with_instances::<Contacts>([base_link])
            .with_rate(tick_rate_hz),
    ]
}

/// Publishes each agent's ground truth onto its autonomy bus. Runs in
/// [`SimulationSet::StateSync`](crate::core::app_state::SimulationSet::StateSync)
/// after [`ground_truth_sync_system`].
///
/// - `truth.tf`: the pose of the agent's `base_link` in the world's datum.
/// - `truth.twist`: `base_link`'s velocity relative to the datum, expressed
///   in `base_link`, rotated out of [`GroundTruthState`]'s world axes by the
///   same pose.
///
/// Stamped with the instant truth describes, [`GroundTruthState::at`], as
/// are the agent's sensor readings and the `now` the host passes into
/// `pipeline.tick`, so `read_fresh(max_age)` comparisons across systems work
/// without skew.
///
/// The body declares both channels, so each write has a slot whether or not
/// a node reads it. A refused write (an undeclared channel or instance, a
/// wrong type) is a host bug, so it warns, once per agent and channel.
pub fn publish_truth_channels_system(
    query: Query<(
        &GroundTruthState,
        &AgentIdComponent,
        &AutonomyPipelineComponent,
    )>,
    tf_tree: Res<TfTree>,
    mut has_logged_first_tick: Local<bool>,
    mut has_warned_no_datum: Local<bool>,
    mut warned_refused: Local<HashSet<(AgentId, ChannelKey)>>,
) {
    // One-shot verification trace: fires exactly once the first time this
    // system actually executes. Used to confirm the schedule wiring is
    // correct (system is in StateSync, run_if(Running) is satisfied) — a
    // silent run cannot otherwise be observed from outside unless a node
    // reads the truth channels. Cheap to leave in; remove if it ever becomes
    // noise. Enable with RUST_LOG=helios_sim::oracle=trace.
    if !*has_logged_first_tick {
        tracing::trace!(
            target: "helios_sim::oracle",
            agent_count = query.iter().len(),
            "publish_truth_channels_system: first tick"
        );
        *has_logged_first_tick = true;
    }

    // The world layout sets the datum before any agent
    // ticks, so a missing datum is a scheduling bug, not a startup race.
    let Some(datum) = tf_tree.datum() else {
        if !*has_warned_no_datum {
            warn!("The world has no datum; ground truth is not published.");
            *has_warned_no_datum = true;
        }
        return;
    };

    for (ground_truth, agent, autonomy_pipeline) in query {
        let now = ground_truth.at;
        let (pose, twist) = truth_messages(ground_truth, &agent.0, datum);

        // Each key is named once, so the warning names the key written.
        let bus = autonomy_pipeline.0.bus();
        let (pose_key, twist_key) = (truth_tf(), truth_twist());
        let pose_result = bus.write_keyed(pose_key.clone(), stamped_now(pose, now));
        let twist_result = bus.write_keyed(twist_key.clone(), stamped_now(twist, now));

        for (key, result) in [(pose_key, pose_result), (twist_key, twist_result)] {
            warn_dropped_truth(&agent.0, key, result, &mut warned_refused);
        }
    }
}

/// `agent`'s truth as the two messages the truth channels carry, stamped
/// with the instant it describes: `base_link`'s pose in `datum`, and its
/// velocity relative to `datum` expressed in `base_link`.
fn truth_messages(
    ground_truth: &GroundTruthState,
    agent: &AgentId,
    datum: &FrameId,
) -> (StampedTransform, StampedTwist<Flu>) {
    let now = ground_truth.at;
    let base_link = FrameId::base_link(agent.clone());
    let body_to_datum = CoreTransform::<Flu, DatumAxes>::from_isometry(ground_truth.pose);
    let datum_to_body = body_to_datum.inverse();

    let pose = StampedTransform {
        parent: datum.clone(),
        child: base_link.clone(),
        stamp: now,
        transform: ErasedTransform::erase(body_to_datum),
    };
    let twist = StampedTwist {
        moving: base_link.clone(),
        relative_to: datum.clone(),
        expressed_in: base_link,
        stamp: now,
        linear: FreeVector::<DatumAxes>::from_raw(ground_truth.linear_velocity)
            .reexpress(&datum_to_body),
        angular: FreeVector::<DatumAxes>::from_raw(ground_truth.angular_velocity)
            .reexpress(&datum_to_body),
    };
    (pose, twist)
}

/// `value` stamped at `now` by the host.
pub(crate) fn stamped_now<T>(value: T, now: MonotonicTime) -> Stamped<T> {
    Stamped {
        value,
        timestamp: now,
        health: Health::Ok,
        producer: HOST_PRODUCER_ID,
    }
}

/// Warns about a refused truth write, once per agent and channel.
pub(crate) fn warn_dropped_truth(
    agent: &AgentId,
    key: ChannelKey,
    result: Result<(), ChannelError>,
    warned_refused: &mut HashSet<(AgentId, ChannelKey)>,
) {
    let Err(error) = result else {
        return;
    };
    if !warned_refused.insert((agent.clone(), key.clone())) {
        return;
    }
    warn!(
        "Truth channel {key} of agent '{agent}' refused a write ({error}); truth is being dropped."
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    /// A body facing north (a quarter turn left of east) and driving north
    /// while yawing left: forward in its own axes, yaw rate unchanged.
    #[test]
    fn the_twist_is_expressed_in_base_link_and_the_pose_in_the_datum() {
        let agent = AgentId::new("car");
        let datum = FrameId::global("datum");
        let ground_truth = GroundTruthState {
            at: MonotonicTime(1.0),
            pose: Isometry3::from_parts(
                Translation3::new(3.0, 4.0, 0.0),
                UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2),
            ),
            linear_velocity: Vector3::new(0.0, 2.0, 0.0),
            angular_velocity: Vector3::new(0.0, 0.0, 0.5),
            ..Default::default()
        };

        let (pose, twist) = truth_messages(&ground_truth, &agent, &datum);

        let base_link = FrameId::base_link(agent);
        assert_eq!((&pose.parent, &pose.child), (&datum, &base_link));
        assert_eq!(
            (
                pose.transform.from_convention(),
                pose.transform.to_convention()
            ),
            (Convention::Flu, Convention::Enu)
        );
        assert_eq!(pose.transform.isometry(), ground_truth.pose);
        assert_eq!(
            (pose.stamp, twist.stamp),
            (ground_truth.at, ground_truth.at)
        );

        assert_eq!(
            (&twist.moving, &twist.relative_to, &twist.expressed_in),
            (&base_link, &datum, &base_link)
        );
        assert!((twist.linear.into_inner() - Vector3::new(2.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((twist.angular.into_inner() - Vector3::new(0.0, 0.0, 0.5)).norm() < 1e-9);
    }

    /// The twist states the datum it is relative to, so the build can refuse
    /// a node reading it beside a pose in another world frame. The pose
    /// names its datum through its instance's parent instead.
    #[test]
    fn the_twist_is_stated_relative_to_the_datum() {
        const TICK_RATE_HZ: f64 = 100.0;
        let datum = FrameId::global("datum");
        let [pose, twist, _contacts] =
            truth_body_channels(&AgentId::new("car"), &datum, TICK_RATE_HZ);

        assert_eq!(pose.reference(), None);
        assert_eq!(twist.reference(), Some(&datum));
    }
}
