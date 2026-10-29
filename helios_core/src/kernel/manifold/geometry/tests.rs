use super::*;

use std::f64::consts::{FRAC_PI_2, PI};

use nalgebra::UnitQuaternion;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde_json::json;

/// Slack for results that went through float arithmetic.
const TOLERANCE: f64 = 1e-12;
/// Slack for round trips through wrap, exp and log.
const ROUND_TRIP_TOLERANCE: f64 = 1e-9;
/// Random cases per property test.
const SAMPLES: usize = 1000;
/// Bound on each random component; ±3 lets a rotation or a torus step
/// approach a half turn, where the short way matters.
const COMPONENT_BOUND: f64 = 3.0;
/// Width of the multi-component kinds under test.
const WIDTH: usize = 3;
/// An angle just short of the seam at ±π, in degrees.
const NEAR_SEAM_DEGREES: f64 = 170.0;
/// The short way between +170° and −170°, across the seam.
const ACROSS_SEAM_DEGREES: f64 = 20.0;
/// Halfway.
const MIDPOINT: f64 = 0.5;

fn width(n: usize) -> NonZeroUsize {
    NonZeroUsize::new(n).expect("test widths are nonzero")
}

fn all_kinds() -> [GeometryKind; 3] {
    [
        GeometryKind::Euclidean { n: width(WIDTH) },
        GeometryKind::Torus { n: width(WIDTH) },
        GeometryKind::Rotation3,
    ]
}

fn random_point(kind: GeometryKind, rng: &mut StdRng) -> Vec<f64> {
    let mut component = || rng.gen_range(-COMPONENT_BOUND..COMPONENT_BOUND);

    match kind {
        GeometryKind::Euclidean { n } => (0..n.get()).map(|_| component()).collect(),
        GeometryKind::Torus { n } => (0..n.get()).map(|_| angle::wrap(component())).collect(),
        GeometryKind::Rotation3 => {
            let axis_angle = Vector3::new(component(), component(), component());
            rotation::to_storage(&UnitQuaternion::from_scaled_axis(axis_angle)).to_vec()
        }
    }
}

/// Length of the step between two points, by the kind's own ⊟. Compares
/// points without caring about wrap or the quaternion's sign.
fn distance(kind: GeometryKind, p: &[f64], q: &[f64]) -> f64 {
    let mut step = vec![0.0; kind.tangent_dim()];
    kind.ominus(p, q, &mut step);

    step.iter().map(|s| s * s).sum::<f64>().sqrt()
}

#[test]
fn dims_match_each_kind() {
    let euclidean = GeometryKind::Euclidean { n: width(WIDTH) };
    let torus = GeometryKind::Torus { n: width(1) };

    assert_eq!(
        (euclidean.storage_dim(), euclidean.tangent_dim()),
        (WIDTH, WIDTH)
    );
    assert_eq!((torus.storage_dim(), torus.tangent_dim()), (1, 1));
    assert_eq!(
        (
            GeometryKind::Rotation3.storage_dim(),
            GeometryKind::Rotation3.tangent_dim()
        ),
        (rotation::STORAGE_DIM, rotation::TANGENT_DIM)
    );
}

#[test]
fn euclidean_steps_add_and_subtract() {
    let kind = GeometryKind::Euclidean { n: width(2) };
    let mut moved = [0.0; 2];
    let mut step = [0.0; 2];

    kind.oplus(&[1.0, -2.0], &[0.5, 4.0], &mut moved);
    kind.ominus(&[1.0, -2.0], &[3.0, 3.0], &mut step);

    assert_eq!(moved, [1.5, 2.0]);
    assert_eq!(step, [-2.0, -5.0]);
}

#[test]
fn torus_difference_crosses_the_seam_the_short_way() {
    // From +170° to −170° is +20° across ±π, not −340° back through 0.
    let kind = GeometryKind::Torus { n: width(1) };
    let mut step = [0.0];

    kind.ominus(
        &[-NEAR_SEAM_DEGREES.to_radians()],
        &[NEAR_SEAM_DEGREES.to_radians()],
        &mut step,
    );

    assert!((step[0] - ACROSS_SEAM_DEGREES.to_radians()).abs() < TOLERANCE);
}

#[test]
fn torus_half_turn_difference_is_plus_pi() {
    // Exactly opposite angles are π apart either way; the kind answers +π.
    let kind = GeometryKind::Torus { n: width(1) };
    let mut step = [0.0];

    kind.ominus(&[FRAC_PI_2], &[-FRAC_PI_2], &mut step);

    assert_eq!(step[0], PI);
}

#[test]
fn torus_step_stays_inside_one_turn() {
    let kind = GeometryKind::Torus { n: width(WIDTH) };
    let mut rng = StdRng::seed_from_u64(0x5152_5354);
    let mut moved = [0.0; WIDTH];

    for _ in 0..SAMPLES {
        let x = random_point(kind, &mut rng);
        let step = random_point(GeometryKind::Euclidean { n: width(WIDTH) }, &mut rng);

        kind.oplus(&x, &step, &mut moved);

        assert!(
            moved.iter().all(|m| -PI < *m && *m <= PI),
            "moved to {moved:?}"
        );
    }
}

#[test]
fn torus_components_wrap_independently() {
    // Only the first component crosses the seam; the others must not move
    // with it.
    let kind = GeometryKind::Torus { n: width(WIDTH) };
    let near_seam = NEAR_SEAM_DEGREES.to_radians();
    let across = ACROSS_SEAM_DEGREES.to_radians();
    let mut moved = [0.0; WIDTH];

    kind.oplus(&[near_seam, 0.0, -1.0], &[across, 0.0, 0.5], &mut moved);

    assert!((moved[0] + near_seam).abs() < TOLERANCE);
    assert_eq!(moved[1], 0.0);
    assert_eq!(moved[2], -0.5);
}

#[test]
fn rotation3_matches_the_rotation_module() {
    let kind = GeometryKind::Rotation3;
    let mut rng = StdRng::seed_from_u64(0x6162_6364);
    let mut moved = [0.0; rotation::STORAGE_DIM];
    let mut step = [0.0; rotation::TANGENT_DIM];

    for _ in 0..SAMPLES {
        let x = random_point(kind, &mut rng);
        let y = random_point(kind, &mut rng);
        let q_x = rotation::from_storage(&x);
        let q_y = rotation::from_storage(&y);

        kind.ominus(&y, &x, &mut step);
        kind.oplus(&x, &step, &mut moved);

        let expected_step = rotation::ominus(&q_y, &q_x);
        let expected_moved = rotation::oplus(&q_x, &expected_step);
        assert_eq!(step, *expected_step.as_slice());
        assert_eq!(moved, rotation::to_storage(&expected_moved));
    }
}

#[test]
fn oplus_undoes_ominus_for_every_kind() {
    let mut rng = StdRng::seed_from_u64(0x7172_7374);

    for kind in all_kinds() {
        let mut step = vec![0.0; kind.tangent_dim()];
        let mut recovered = vec![0.0; kind.storage_dim()];

        for _ in 0..SAMPLES {
            let x = random_point(kind, &mut rng);
            let y = random_point(kind, &mut rng);

            kind.ominus(&y, &x, &mut step);
            kind.oplus(&x, &step, &mut recovered);

            assert!(
                distance(kind, &recovered, &y) < ROUND_TRIP_TOLERANCE,
                "{kind:?}"
            );
        }
    }
}

#[test]
fn interpolate_returns_the_endpoints_bit_for_bit() {
    let mut rng = StdRng::seed_from_u64(0x8182_8384);

    for kind in all_kinds() {
        let mut out = vec![0.0; kind.storage_dim()];

        for _ in 0..SAMPLES {
            let a = random_point(kind, &mut rng);
            let b = random_point(kind, &mut rng);

            kind.interpolate(&a, &b, 0.0, &mut out);
            assert_eq!(out, a, "{kind:?} at the start");
            kind.interpolate(&a, &b, 1.0, &mut out);
            assert_eq!(out, b, "{kind:?} at the end");
        }
    }
}

#[test]
fn interpolate_takes_a_scaled_step_for_every_kind() {
    // Each arm writes a ⊞ (t · (b ⊟ a)) out by hand; it must agree with
    // composing the kind's own ominus and oplus.
    let mut rng = StdRng::seed_from_u64(0x9192_9394);

    for kind in all_kinds() {
        let mut out = vec![0.0; kind.storage_dim()];
        let mut step = vec![0.0; kind.tangent_dim()];
        let mut expected = vec![0.0; kind.storage_dim()];

        for _ in 0..SAMPLES {
            let a = random_point(kind, &mut rng);
            let b = random_point(kind, &mut rng);
            let t = rng.gen_range(0.0..1.0);

            kind.interpolate(&a, &b, t, &mut out);
            kind.ominus(&b, &a, &mut step);
            step.iter_mut().for_each(|s| *s *= t);
            kind.oplus(&a, &step, &mut expected);

            assert!(distance(kind, &out, &expected) < TOLERANCE, "{kind:?}");
        }
    }
}

#[test]
fn torus_midpoint_across_the_seam_is_the_half_turn() {
    // Halfway from +170° to −170° is 180°, not 0°.
    let kind = GeometryKind::Torus { n: width(1) };
    let mut out = [0.0];

    kind.interpolate(
        &[NEAR_SEAM_DEGREES.to_radians()],
        &[-NEAR_SEAM_DEGREES.to_radians()],
        MIDPOINT,
        &mut out,
    );

    assert!(
        distance(kind, &out, &[PI]) < TOLERANCE,
        "midpoint was {}",
        out[0]
    );
}

#[test]
fn rotation_midpoint_is_equidistant_from_both_ends() {
    let kind = GeometryKind::Rotation3;
    let mut rng = StdRng::seed_from_u64(0xA1A2_A3A4);
    let mut mid = [0.0; rotation::STORAGE_DIM];

    for _ in 0..SAMPLES {
        let a = random_point(kind, &mut rng);
        let b = random_point(kind, &mut rng);

        kind.interpolate(&a, &b, MIDPOINT, &mut mid);

        let half = distance(kind, &a, &b) * MIDPOINT;
        assert!((distance(kind, &a, &mid) - half).abs() < ROUND_TRIP_TOLERANCE);
        assert!((distance(kind, &mid, &b) - half).abs() < ROUND_TRIP_TOLERANCE);
    }
}

#[test]
fn interpolate_between_opposite_rotations_stays_finite() {
    // A half turn apart, the short way is ambiguous; the kind must still
    // pick one and land a quarter turn from each end.
    let kind = GeometryKind::Rotation3;
    let identity = rotation::to_storage(&UnitQuaternion::identity());
    let half_turn = rotation::to_storage(&UnitQuaternion::from_scaled_axis(Vector3::new(
        0.0, 0.0, PI,
    )));
    let mut mid = [0.0; rotation::STORAGE_DIM];

    kind.interpolate(&identity, &half_turn, MIDPOINT, &mut mid);

    assert!(mid.iter().all(|m| m.is_finite()));
    assert!((distance(kind, &identity, &mid) - FRAC_PI_2).abs() < ROUND_TRIP_TOLERANCE);
    assert!((distance(kind, &mid, &half_turn) - FRAC_PI_2).abs() < ROUND_TRIP_TOLERANCE);
}

#[test]
fn serialized_shape_is_tagged_by_kind() {
    // The shape is hashed into content identities; changing it changes them.
    let cases = [
        (
            GeometryKind::Euclidean { n: width(WIDTH) },
            json!({ "kind": "euclidean", "n": WIDTH }),
        ),
        (
            GeometryKind::Torus { n: width(1) },
            json!({ "kind": "torus", "n": 1 }),
        ),
        (GeometryKind::Rotation3, json!({ "kind": "rotation3" })),
    ];

    for (kind, shape) in cases {
        assert_eq!(serde_json::to_value(kind).expect("serializes"), shape);
        assert_eq!(
            serde_json::from_value::<GeometryKind>(shape).expect("reads"),
            kind
        );
    }
}

#[test]
fn toml_reads_the_tagged_form() {
    #[derive(Deserialize)]
    struct Column {
        geometry: GeometryKind,
    }

    let column: Column =
        toml::from_str(r#"geometry = { kind = "torus", n = 1 }"#).expect("reads");

    assert_eq!(column.geometry, GeometryKind::Torus { n: width(1) });
}

#[test]
fn a_zero_width_kind_is_refused_on_read() {
    let refused = serde_json::from_value::<GeometryKind>(json!({ "kind": "torus", "n": 0 }));

    assert!(refused.is_err());
}
