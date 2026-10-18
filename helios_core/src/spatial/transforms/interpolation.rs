//! A pose between two poses: the one rule every reader that lines up poses in
//! time shares, so a lookup in the transform tree and a pose graded against
//! truth interpolate alike.

use nalgebra::{Isometry3, Translation3};

/// Below this sine of the angle between the two rotations' quaternions,
/// slerp declines: the shortest arc between them is ill-defined.
const SLERP_ANTIPODAL_EPS: f64 = 1e-6;

/// The pose `fraction` of the way from `from` to `to`, `fraction` clamped to
/// `[0, 1]`. Translation is Euclidean, so it is a straight-line lerp. Rotation
/// is not: a componentwise average of two quaternions is not a rotation, so it
/// walks the shortest arc with slerp. Where that arc is ill-defined, the blend
/// snaps to the nearer endpoint instead.
pub fn interpolate_pose(
    from: &Isometry3<f64>,
    to: &Isometry3<f64>,
    fraction: f64,
) -> Isometry3<f64> {
    let fraction = fraction.clamp(0.0, 1.0);
    let translation = from
        .translation
        .vector
        .lerp(&to.translation.vector, fraction);
    let rotation = from
        .rotation
        .try_slerp(&to.rotation, fraction, SLERP_ANTIPODAL_EPS)
        .unwrap_or(if fraction < 0.5 {
            from.rotation
        } else {
            to.rotation
        });
    Isometry3::from_parts(Translation3::from(translation), rotation)
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    const TOL: f64 = 1e-12;

    /// A pose at `x` along +X, yawed `yaw` about +Z.
    fn pose(x: f64, yaw: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(x, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
        )
    }

    fn close(a: &Isometry3<f64>, b: &Isometry3<f64>) -> bool {
        (a.translation.vector - b.translation.vector).norm() < TOL
            && a.rotation.angle_to(&b.rotation) < TOL
    }

    #[test]
    fn the_midpoint_lerps_translation_and_slerps_rotation() {
        let between = interpolate_pose(&pose(0.0, 0.0), &pose(10.0, FRAC_PI_2), 0.5);

        assert!(close(&between, &pose(5.0, FRAC_PI_4)), "{between:?}");
    }

    #[test]
    fn the_ends_are_the_poses_and_a_fraction_past_them_is_clamped() {
        let (from, to) = (pose(1.0, 0.2), pose(3.0, 0.6));

        assert!(close(&interpolate_pose(&from, &to, 0.0), &from));
        assert!(close(&interpolate_pose(&from, &to, 1.0), &to));
        assert!(close(&interpolate_pose(&from, &to, -1.0), &from));
        assert!(close(&interpolate_pose(&from, &to, 2.0), &to));
    }
}
