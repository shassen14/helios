//! A step under real physics: Avian in `FixedUpdate`, the set chain, the run
//! clock and the truth sync, as the host wires them, at a frame that runs
//! one step and at one that runs a batch.

use super::*;
use crate::core::app_state::{AppState, SimulationSet};
use crate::core::components::GroundTruthState;
use crate::core::ground_truth_sync_system;
use crate::core::run_clock::{RunClock, RunClockPlugin};
use crate::core::simulation_setup::configure_fixed_update_sets;
use crate::core::transforms::{bevy_transform_to_transform_bevy, FromBevy};

use helios_core::prelude::MonotonicTime;
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::transforms::Transform as CoreTransform;

use avian3d::prelude::{
    Collider, ConstantForce, Gravity, LinearVelocity, Position, RigidBody, Rotation, SpatialQuery,
    SpatialQueryFilter,
};
use bevy::ecs::system::RunSystemOnce;
use bevy::state::app::StatesPlugin;
use bevy::time::TimeUpdateStrategy;
use std::time::Duration;

/// The scenario's 400 Hz tick.
const STEP: Duration = Duration::from_micros(2500);
/// A frame that runs one step, and the batch a headless run takes at 400 Hz.
const SINGLE: u32 = 1;
const BATCH: u32 = 20;
/// Steps each case records: two whole batches.
const STEPS: usize = 2 * BATCH as usize;
/// A body moving this fast (Bevy x) moves 12.5 mm per step, far above the
/// tolerance, so a value one step stale can't pass.
const SPEED_M_S: f32 = 5.0;
/// A sensor mount above the body and ahead of it, turned a quarter.
const MOUNT_OFFSET_M: Vec3 = Vec3::new(0.3, 0.8, -0.2);
/// f32 positions after a few dozen steps (metres).
const TOLERANCE_M: f32 = 1e-5;

/// Marks the moving body; its one child is the mount.
#[derive(Component)]
struct Body;

#[derive(Component)]
struct Mount;

/// What the step's last set saw.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Seen {
    now: Option<MonotonicTime>,
    at: MonotonicTime,
    position: Vec3,
    rotation: Quat,
    truth: Vec3,
    mount: Vec3,
}

#[derive(Resource, Default)]
struct Record(Vec<Seen>);

/// What a step leaves in `Validation`, where every reader of the step has
/// run.
fn record(
    body: Single<(&Position, &Rotation, &GroundTruthState), With<Body>>,
    mount: Single<&GlobalTransform, With<Mount>>,
    clock: RunClock,
    mut record: ResMut<Record>,
) {
    let (position, rotation, truth) = *body;
    let at = truth.at;
    let truth = truth.pose.translation.vector;
    record.0.push(Seen {
        now: clock.now(),
        at,
        position: position.0,
        rotation: rotation.0,
        truth: Vec3::new(truth.x as f32, truth.y as f32, truth.z as f32),
        mount: mount.translation(),
    });
}

/// An app stepping `steps_per_update` fixed steps a frame, wired as the host
/// wires physics, the clock and the set chain, still loading.
fn stepping_app(steps_per_update: u32) -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        StatesPlugin,
        AssetPlugin::default(),
        bevy::mesh::MeshPlugin,
        PhysicsStepPlugin,
        RunClockPlugin,
    ))
    .init_state::<AppState>()
    .insert_resource(Time::<Fixed>::from_duration(STEP))
    .insert_resource(TimeUpdateStrategy::FixedTimesteps(steps_per_update));
    configure_fixed_update_sets(&mut app);
    // `App::run` would call these; stepping by hand must, since Avian adds
    // some of its resources only when plugins finish.
    app.finish();
    app.cleanup();
    app
}

/// Sets the app to enter `Running` on its next update. Not before its first:
/// physics pauses at startup, after the state machine's first transition.
fn enter_running(app: &mut App) {
    app.world_mut()
        .resource_mut::<NextState<AppState>>()
        .set(AppState::Running);
}

/// An app stepping `steps_per_update` fixed steps a frame, about to run,
/// with the body moving at a constant velocity.
fn running_app(steps_per_update: u32) -> App {
    let mut app = stepping_app(steps_per_update);
    app.init_resource::<Record>();
    app.add_systems(
        FixedUpdate,
        (
            ground_truth_sync_system
                .in_set(SimulationSet::StateSync)
                .run_if(in_state(AppState::Running)),
            record
                .in_set(SimulationSet::Validation)
                .run_if(in_state(AppState::Running)),
        ),
    );

    app.world_mut()
        .spawn((
            Body,
            RigidBody::Kinematic,
            LinearVelocity(Vec3::X * SPEED_M_S),
            Transform::default(),
            GroundTruthState::default(),
        ))
        .with_child((
            Mount,
            Transform::from_translation(MOUNT_OFFSET_M)
                .with_rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)),
        ));

    app.update();
    enter_running(&mut app);
    app
}

/// What each of the run's first [`STEPS`] steps saw.
fn steps_seen(steps_per_update: u32) -> Vec<Seen> {
    let mut app = running_app(steps_per_update);
    while app.world().resource::<Record>().0.len() < STEPS {
        app.update();
    }
    app.world().resource::<Record>().0[..STEPS].to_vec()
}

/// Truth's position, in the world ENU axes it holds, for a body at
/// `position` turned by `rotation` (Bevy).
fn truth_for(position: Vec3, rotation: Quat) -> Vec3 {
    let world: CoreTransform<Flu, Enu> = bevy_transform_to_transform_bevy(
        Transform::from_translation(position).with_rotation(rotation),
    )
    .from_bevy();
    let translation = world.into_inner().translation.vector;
    Vec3::new(
        translation.x as f32,
        translation.y as f32,
        translation.z as f32,
    )
}

fn assert_near(actual: Vec3, expected: Vec3, what: &str, step: usize) {
    assert!(
        actual.distance(expected) < TOLERANCE_M,
        "step {step}: {what} {actual} is not {expected}"
    );
}

/// Truth and the mount's pose describe the body where physics left it this
/// step, on every step of a batch and not only its first, and truth names
/// the step's instant.
#[test]
fn every_value_in_a_step_describes_the_same_instant() {
    for steps_per_update in [SINGLE, BATCH] {
        for (step, seen) in steps_seen(steps_per_update).into_iter().enumerate() {
            assert_eq!(
                Some(seen.at),
                seen.now,
                "{steps_per_update} step(s) per update, step {step}: truth's instant"
            );
            let case = format!("{steps_per_update} step(s) per update, truth");
            assert_near(
                seen.truth,
                truth_for(seen.position, seen.rotation),
                &case,
                step,
            );
            let mount = seen.position + seen.rotation * MOUNT_OFFSET_M;
            let case = format!("{steps_per_update} step(s) per update, mount");
            assert_near(seen.mount, mount, &case, step);
            // Step `k` (from 1) has carried the body `k` steps from its spawn.
            let travelled = Vec3::X * SPEED_M_S * STEP.as_secs_f32() * (step + 1) as f32;
            let case = format!("{steps_per_update} step(s) per update, body");
            assert_near(seen.position, travelled, &case, step);
        }
    }
}

/// The step is the same whichever frame it falls in.
#[test]
fn how_many_steps_a_frame_runs_changes_no_step() {
    assert_eq!(steps_seen(SINGLE), steps_seen(BATCH));
}

/// A ground slab whose top face is at Bevy y = 0, and a ball resting just
/// above it, as a scene builds them before the run.
const BALL_HEIGHT_M: f32 = 1.0;
const BALL_RADIUS_M: f32 = 0.25;
const GROUND_HALF_EXTENT_M: f32 = 10.0;
const GROUND_HALF_THICKNESS_M: f32 = 0.5;
/// Updates a scene can sit through before the run starts.
const PRE_RUN_UPDATES: usize = 5;

fn spawn_ball_over_ground(app: &mut App) -> Entity {
    app.world_mut().spawn((
        RigidBody::Static,
        Collider::cuboid(
            2.0 * GROUND_HALF_EXTENT_M,
            2.0 * GROUND_HALF_THICKNESS_M,
            2.0 * GROUND_HALF_EXTENT_M,
        ),
        Transform::from_xyz(0.0, -GROUND_HALF_THICKNESS_M, 0.0),
    ));
    app.world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::sphere(BALL_RADIUS_M),
            Transform::from_xyz(0.0, BALL_HEIGHT_M, 0.0),
        ))
        .id()
}

fn height(app: &App, body: Entity) -> f32 {
    app.world().get::<Position>(body).expect("a body").y
}

/// Bodies spawned while the scene builds don't fall before the run, however
/// many steps those frames run, and the world is already there to cast rays
/// into; the run's first step moves them.
#[test]
fn physics_starts_with_the_run() {
    let mut app = stepping_app(BATCH);
    let ball = spawn_ball_over_ground(&mut app);

    for _ in 0..PRE_RUN_UPDATES {
        app.update();
    }
    assert_eq!(
        height(&app, ball),
        BALL_HEIGHT_M,
        "the ball fell before the run"
    );

    let below = app
        .world_mut()
        .run_system_once(|spatial: SpatialQuery| {
            spatial.cast_ray(
                Vec3::new(GROUND_HALF_EXTENT_M / 2.0, BALL_HEIGHT_M, 0.0),
                Dir3::NEG_Y,
                2.0 * BALL_HEIGHT_M,
                true,
                &SpatialQueryFilter::default(),
            )
        })
        .expect("the cast runs");
    let hit = below.expect("a ray cast before any step hits the ground");
    assert!(
        (hit.distance - BALL_HEIGHT_M).abs() < TOLERANCE_M,
        "the ground is {} below, not {BALL_HEIGHT_M}",
        hit.distance
    );

    enter_running(&mut app);
    app.update();
    assert!(
        height(&app, ball) < BALL_HEIGHT_M,
        "the run's first steps moved nothing"
    );
}

/// The velocity the step's `StateSync` saw.
#[derive(Resource, Default)]
struct SyncedVelocity(Option<Vec3>);

const PUSH_N: f32 = 10.0;

/// A force applied in `Actuation` through `Commands`, as the vehicle plugin
/// applies its wrench, acts in the same step: physics shares the schedule,
/// so only the sync point between the sets carries it to the solver.
#[test]
fn a_force_applied_in_actuation_acts_in_the_same_step() {
    let mut app = stepping_app(SINGLE);
    app.insert_resource(Gravity::ZERO)
        .init_resource::<SyncedVelocity>()
        .add_systems(
            FixedUpdate,
            (
                (|mut commands: Commands, bodies: Query<Entity, With<Body>>| {
                    for body in &bodies {
                        commands
                            .entity(body)
                            .insert(ConstantForce::new(PUSH_N, 0.0, 0.0));
                    }
                })
                .in_set(SimulationSet::Actuation)
                .run_if(in_state(AppState::Running)),
                (|velocity: Single<&LinearVelocity, With<Body>>,
                  mut synced: ResMut<SyncedVelocity>| {
                    synced.0.get_or_insert(velocity.0);
                })
                .in_set(SimulationSet::StateSync)
                .run_if(in_state(AppState::Running)),
            ),
        );
    app.world_mut().spawn((
        Body,
        RigidBody::Dynamic,
        Collider::sphere(BALL_RADIUS_M),
        Transform::default(),
    ));

    // The first frame runs startup, which pauses physics, and advances no
    // time.
    app.update();
    enter_running(&mut app);
    app.update();

    let first_step = app.world().resource::<SyncedVelocity>().0;
    let first_step = first_step.expect("the run stepped");
    assert!(
        first_step.x > 0.0,
        "the force pushed nothing in its own step: velocity {first_step}"
    );
}
