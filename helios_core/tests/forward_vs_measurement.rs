//! The one place `sensors/` and `estimation/measurement/` meet.
//!
//! Each state sensor is modelled twice in `helios_core`: a forward (truth) model
//! in `sensors/` that answers *what the sensor actually does*, and a
//! [`MeasurementModel`] in `estimation/measurement/` that answers *what the
//! filter believes it does*. Production code in the two modules never references
//! the other — conflating them is the inverse crime the crate rules forbid. They
//! are allowed to touch in exactly one spot: this test, which pins the filter's
//! `h(x)` as a correct linearization of the truth model.
//!
//! The check per sensor: with noise and bias switched off (the forward model's
//! `ideal()`, which adds neither), the forward reading and the filter's
//! `predict_measurement()` agree to numerical tolerance. Each case is driven
//! from one shared ground-truth kinematic state; the two models take that truth
//! in different conventions (the forward model in world axes plus a single
//! sensor-from-world rotation; the filter in body-frame layout slots plus a TF
//! extrinsic), so the test's real work is deriving both input sets from the same
//! truth. Cases are drawn from a seeded RNG, so the property is checked over
//! many random poses, rates, mounts, and fields while staying reproducible.

use helios_core::data::primitives::FrameHandle;
use helios_core::estimation::measurement::accelerometer::SpecificForceModel;
use helios_core::estimation::measurement::gps::GpsPositionModel;
use helios_core::estimation::measurement::gyroscope::AngularRateModel;
use helios_core::estimation::measurement::magnetometer::MagneticFieldModel;
use helios_core::estimation::measurement::MeasurementModel;
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
use helios_core::ports::TfProvider;
use helios_core::sensors::accelerometer::AccelerometerModel;
use helios_core::sensors::gps::GpsModel;
use helios_core::sensors::gyroscope::GyroscopeModel;
use helios_core::sensors::magnetometer::MagnetometerModel;

use std::f64::consts::{FRAC_PI_2, PI};

use nalgebra::{DVector, Isometry3, Translation3, UnitQuaternion, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

const AGENT: FrameHandle = FrameHandle(1);
const SENSOR: FrameHandle = FrameHandle(2);

/// Random cases per sensor. Large enough to exercise the frame conventions
/// across the pose/rate space; small enough to stay a fast unit-style test.
const CASES: usize = 500;

/// Any positive stddev builds a valid forward model; `ideal()` never reads it,
/// so its value is irrelevant to the comparison.
const UNIT_STDDEV: Vector3<f64> = Vector3::new(1.0, 1.0, 1.0);

/// Rotations and cross products in `f64` leave the two paths agreeing to ~1e-12
/// at these magnitudes; 1e-9 is a comfortable ceiling.
const TOL: f64 = 1e-9;

// A TF tree that reports one fixed extrinsic — the sensor's pose expressed in
// body axes, which is what every measurement model reads out of `get_transform`.
struct SensorInBody(Isometry3<f64>);

impl TfProvider for SensorInBody {
    fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
        Some(self.0)
    }

    fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
        Some(Isometry3::identity())
    }
}

#[test]
fn gps_forward_matches_filter_prediction() {
    let mut rng = StdRng::seed_from_u64(0x6759_0001);
    let forward = GpsModel::new(Vector3::zeros(), UNIT_STDDEV).unwrap();
    let filter = GpsPositionModel {
        agent_handle: AGENT,
        sensor_handle: SENSOR,
    };

    for _ in 0..CASES {
        let body_position = rand_vec3(&mut rng, 100.0);
        let q_body_to_world = rand_quat(&mut rng);
        let lever_body = rand_vec3(&mut rng, 2.0);
        // GPS ignores the mount rotation; a random one guards against that
        // assumption silently breaking.
        let extrinsic = Isometry3::from_parts(Translation3::from(lever_body), rand_quat(&mut rng));

        // Filter: body position from the state, antenna offset from the TF tree.
        let state = make_state(
            body_position,
            q_body_to_world,
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let predicted = filter
            .predict_measurement(&state, Some(&SensorInBody(extrinsic)))
            .unwrap();

        // Forward: the antenna observes its own world position directly.
        let antenna_world = body_position + q_body_to_world * lever_body;
        let ideal = forward.ideal(antenna_world);

        assert_agrees("gps", ideal, &predicted);
    }
}

#[test]
fn gyroscope_forward_matches_filter_prediction() {
    let mut rng = StdRng::seed_from_u64(0x6759_0002);
    let forward = GyroscopeModel::new(Vector3::zeros(), UNIT_STDDEV).unwrap();
    let filter = AngularRateModel {
        agent_handle: AGENT,
        sensor_handle: SENSOR,
    };

    for _ in 0..CASES {
        let q_body_to_world = rand_quat(&mut rng);
        let omega_body = rand_vec3(&mut rng, 10.0);
        // The extrinsic's rotation maps sensor axes -> body axes.
        let extrinsic =
            Isometry3::from_parts(Translation3::from(rand_vec3(&mut rng, 2.0)), rand_quat(&mut rng));

        let state = make_state(
            Vector3::zeros(),
            q_body_to_world,
            omega_body,
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let predicted = filter
            .predict_measurement(&state, Some(&SensorInBody(extrinsic)))
            .unwrap();

        let ideal = forward.ideal(
            q_body_to_world * omega_body,
            sensor_from_world(&extrinsic, q_body_to_world),
        );

        assert_agrees("gyroscope", ideal, &predicted);
    }
}

#[test]
fn accelerometer_forward_matches_filter_prediction() {
    let gravity_world = Vector3::new(0.0, 0.0, -9.81);
    let mut rng = StdRng::seed_from_u64(0x6759_0003);
    let forward = AccelerometerModel::new(Vector3::zeros(), UNIT_STDDEV).unwrap();
    let filter = SpecificForceModel {
        agent_handle: AGENT,
        sensor_handle: SENSOR,
        gravity_world,
    };

    for _ in 0..CASES {
        let q_body_to_world = rand_quat(&mut rng);
        let accel_body = rand_vec3(&mut rng, 20.0);
        let omega_body = rand_vec3(&mut rng, 10.0);
        let alpha_body = rand_vec3(&mut rng, 10.0);
        let lever_body = rand_vec3(&mut rng, 2.0);
        let extrinsic = Isometry3::from_parts(Translation3::from(lever_body), rand_quat(&mut rng));

        // Filter: kinematics from body-frame state slots, lever arm from TF.
        let state = make_state(
            rand_vec3(&mut rng, 100.0),
            q_body_to_world,
            omega_body,
            accel_body,
            alpha_body,
        );
        let predicted = filter
            .predict_measurement(&state, Some(&SensorInBody(extrinsic)))
            .unwrap();

        // Forward: the same kinematics rotated into world axes. Rotation
        // preserves the cross products, so the two lever-arm paths coincide.
        let ideal = forward.ideal(
            q_body_to_world * accel_body,
            gravity_world,
            q_body_to_world * omega_body,
            q_body_to_world * alpha_body,
            q_body_to_world * lever_body,
            sensor_from_world(&extrinsic, q_body_to_world),
        );

        assert_agrees("accelerometer", ideal, &predicted);
    }
}

/// The magnetometer pair agrees **only under an identity mount rotation**, so
/// this case pins the sensor frame to the body frame.
///
/// `MagnetometerModel::ideal` reports the field in the *sensor* frame, but
/// `MagneticFieldModel::predict_measurement` predicts it in the *body* frame and
/// ignores the sensor extrinsic entirely. Under a rotated mount the two diverge —
/// a genuine asymmetry with the gyroscope, which does carry the mount rotation.
/// See §4.6 of `docs/notes/host_brain_boundary.md`: whether the filter's model
/// should also apply the sensor rotation is an open decision. When it is settled
/// one way, this test either gains a random mount (fix applied) or keeps the
/// identity restriction and documents the divergence (fix declined).
#[test]
fn magnetometer_forward_matches_filter_prediction_identity_mount() {
    let mut rng = StdRng::seed_from_u64(0x6759_0004);

    for _ in 0..CASES {
        let q_body_to_world = rand_quat(&mut rng);
        let declination = rng.gen_range(-PI..PI);
        let inclination = rng.gen_range(-FRAC_PI_2..FRAC_PI_2);
        let magnitude = rng.gen_range(20.0..70.0);
        let field_world = reference_field_enu(declination, inclination, magnitude);

        let forward = MagnetometerModel::from_reference_field(
            declination,
            inclination,
            magnitude,
            Vector3::zeros(),
            UNIT_STDDEV,
        )
        .unwrap();
        let filter = MagneticFieldModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            world_magnetic_field: field_world,
        };

        // The filter needs no TF for the magnetometer.
        let state = make_state(
            Vector3::zeros(),
            q_body_to_world,
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let predicted = filter.predict_measurement(&state, None).unwrap();

        // Identity mount: sensor-from-world collapses to body-from-world.
        let ideal = forward.ideal(q_body_to_world.inverse());

        assert_agrees("magnetometer", ideal, &predicted);
    }
}

/// The single sensor-from-world rotation the forward models take, composed from
/// the two rotations the filter uses separately: the mount (`body`←`sensor`,
/// stored in the extrinsic) and the body orientation (`world`←`body`, from
/// state). `q_sensor_from_world = q_sensor_from_body * q_body_from_world`.
fn sensor_from_world(
    extrinsic: &Isometry3<f64>,
    q_body_to_world: UnitQuaternion<f64>,
) -> UnitQuaternion<f64> {
    extrinsic.rotation.inverse() * q_body_to_world.inverse()
}

/// Mirrors `MagnetometerModel::from_reference_field`'s ENU resolution so the
/// filter's model can be fed the identical world field vector.
fn reference_field_enu(declination: f64, inclination: f64, magnitude: f64) -> Vector3<f64> {
    Vector3::new(
        magnitude * inclination.cos() * declination.sin(),
        magnitude * inclination.cos() * declination.cos(),
        -magnitude * inclination.sin(),
    )
}

/// Builds a filter state carrying every slot the four models read: world
/// position, the body→world orientation, and the body-frame angular velocity,
/// linear acceleration, and angular acceleration. Each group is laid out
/// contiguously so `get_vector3` / `get_orientation` can find it.
fn make_state(
    position_world: Vector3<f64>,
    q_body_to_world: UnitQuaternion<f64>,
    angular_vel_body: Vector3<f64>,
    linear_accel_body: Vector3<f64>,
    angular_accel_body: Vector3<f64>,
) -> FrameAwareState {
    let world = FrameId::World;
    let body = FrameId::Body(AGENT);
    let layout = vec![
        StateVariable::Px(world.clone()),
        StateVariable::Py(world.clone()),
        StateVariable::Pz(world.clone()),
        StateVariable::Qx(body.clone(), world.clone()),
        StateVariable::Qy(body.clone(), world.clone()),
        StateVariable::Qz(body.clone(), world.clone()),
        StateVariable::Qw(body.clone(), world.clone()),
        StateVariable::Wx(body.clone()),
        StateVariable::Wy(body.clone()),
        StateVariable::Wz(body.clone()),
        StateVariable::Ax(body.clone()),
        StateVariable::Ay(body.clone()),
        StateVariable::Az(body.clone()),
        StateVariable::Alphax(body.clone()),
        StateVariable::Alphay(body.clone()),
        StateVariable::Alphaz(body.clone()),
    ];
    let mut state = FrameAwareState::new(layout, 1.0, 0.0);

    set_vec3(&mut state, StateVariable::Px(world.clone()), position_world);
    let q = q_body_to_world.quaternion();
    state.set_variable(&StateVariable::Qx(body.clone(), world.clone()), q.i);
    state.set_variable(&StateVariable::Qy(body.clone(), world.clone()), q.j);
    state.set_variable(&StateVariable::Qz(body.clone(), world.clone()), q.k);
    state.set_variable(&StateVariable::Qw(body.clone(), world.clone()), q.w);
    set_vec3(&mut state, StateVariable::Wx(body.clone()), angular_vel_body);
    set_vec3(&mut state, StateVariable::Ax(body.clone()), linear_accel_body);
    set_vec3(&mut state, StateVariable::Alphax(body.clone()), angular_accel_body);

    state
}

/// Writes a 3-vector into the `x`/`y`/`z` slots that follow `x_variable` in the
/// layout, matching the contiguity `get_vector3` expects.
fn set_vec3(state: &mut FrameAwareState, x_variable: StateVariable, value: Vector3<f64>) {
    let (y_variable, z_variable) = match &x_variable {
        StateVariable::Px(id) => (StateVariable::Py(id.clone()), StateVariable::Pz(id.clone())),
        StateVariable::Wx(id) => (StateVariable::Wy(id.clone()), StateVariable::Wz(id.clone())),
        StateVariable::Ax(id) => (StateVariable::Ay(id.clone()), StateVariable::Az(id.clone())),
        StateVariable::Alphax(id) => (
            StateVariable::Alphay(id.clone()),
            StateVariable::Alphaz(id.clone()),
        ),
        other => panic!("set_vec3 called with a non-vector start variable: {other:?}"),
    };
    state.set_variable(&x_variable, value.x);
    state.set_variable(&y_variable, value.y);
    state.set_variable(&z_variable, value.z);
}

fn rand_vec3(rng: &mut StdRng, scale: f64) -> Vector3<f64> {
    Vector3::new(
        rng.gen_range(-scale..scale),
        rng.gen_range(-scale..scale),
        rng.gen_range(-scale..scale),
    )
}

fn rand_quat(rng: &mut StdRng) -> UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(
        rng.gen_range(-PI..PI),
        rng.gen_range(-PI..PI),
        rng.gen_range(-PI..PI),
    )
}

fn assert_agrees(sensor: &str, forward: Vector3<f64>, predicted: &DVector<f64>) {
    assert_eq!(
        predicted.len(),
        3,
        "{sensor}: filter prediction is not 3-dimensional"
    );
    let predicted = Vector3::new(predicted[0], predicted[1], predicted[2]);
    let error = (forward - predicted).norm();
    assert!(
        error < TOL,
        "{sensor}: forward={forward:?} filter={predicted:?} error={error:e}"
    );
}
