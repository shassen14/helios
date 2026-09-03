use super::frame_types::{EnuBodyPose, EnuVector};

use helios_core::control::commands::BodyTwist;
use helios_core::frames::quantities::FluVector;

use nalgebra::UnitQuaternion;

/// Rotate a world-ENU twist into the body's FLU frame.
///
/// Ground truth reports linear and angular velocity in world ENU; a body-frame
/// plant wants them as an FLU [`BodyTwist`]. The body pose's rotation carries FLU
/// body axes into ENU, so its inverse carries an ENU vector back into FLU. A
/// twist is two free vectors sharing that frame, so the linear and angular halves
/// ride the *same* rotation — no translation (free vectors), no axis swap (both
/// frames are right-handed; that swap is Bevy-only and lives in `bevy_bridge`).
pub fn enu_twist_to_body_flu(
    pose: EnuBodyPose,
    linear_enu: EnuVector,
    angular_enu: EnuVector,
) -> BodyTwist {
    let enu_to_flu: UnitQuaternion<f64> = pose.0.rotation.inverse();
    BodyTwist::new(
        FluVector::from_raw(enu_to_flu * linear_enu.0),
        FluVector::from_raw(enu_to_flu * angular_enu.0),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::{Isometry3, Translation3, Vector3};
    use std::f64::consts::FRAC_PI_2;

    // Floats survive one quaternion rotation, so exact equality is unsafe; compare
    // components within a tight tolerance instead.
    const EPS: f64 = 1e-12;

    fn assert_flu(actual: FluVector, expected: [f64; 3]) {
        assert!(
            (actual.x() - expected[0]).abs() < EPS
                && (actual.y() - expected[1]).abs() < EPS
                && (actual.z() - expected[2]).abs() < EPS,
            "expected {expected:?}, got [{}, {}, {}]",
            actual.x(),
            actual.y(),
            actual.z(),
        );
    }

    fn body_pose(rotation: UnitQuaternion<f64>) -> EnuBodyPose {
        // A non-zero translation that the free-vector rotation must ignore.
        EnuBodyPose(Isometry3::from_parts(
            Translation3::new(10.0, -5.0, 3.0),
            rotation,
        ))
    }

    // Heading East (identity pose): FLU axes coincide with ENU, so both halves of
    // the twist pass through unchanged — and the pose translation is ignored.
    #[test]
    fn identity_pose_passes_enu_through() {
        let twist = enu_twist_to_body_flu(
            body_pose(UnitQuaternion::identity()),
            EnuVector(Vector3::new(1.0, 2.0, 3.0)),
            EnuVector(Vector3::new(4.0, 5.0, 6.0)),
        );

        assert_flu(twist.linear(), [1.0, 2.0, 3.0]);
        assert_flu(twist.angular(), [4.0, 5.0, 6.0]);
    }

    // Body yawed +90° (Forward = ENU North). A world velocity due East is, from the
    // body's view, motion to its right: FLU (0, -1, 0). This is the linear half.
    #[test]
    fn yaw_ninety_maps_east_to_body_right() {
        let north = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        let twist = enu_twist_to_body_flu(
            body_pose(north),
            EnuVector(Vector3::new(1.0, 0.0, 0.0)),
            EnuVector(Vector3::zeros()),
        );

        assert_flu(twist.linear(), [0.0, -1.0, 0.0]);
    }

    // The angular half rides the identical rotation, not a passthrough: under the
    // same North heading, an angular rate about world East becomes a rate about the
    // body's right axis, FLU (0, -1, 0) — while a pure world-yaw rate (about Up,
    // the rotation's own axis) is invariant.
    #[test]
    fn angular_rides_the_same_rotation() {
        let north = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);

        let about_east = enu_twist_to_body_flu(
            body_pose(north),
            EnuVector(Vector3::zeros()),
            EnuVector(Vector3::new(1.0, 0.0, 0.0)),
        );
        assert_flu(about_east.angular(), [0.0, -1.0, 0.0]);

        let about_up = enu_twist_to_body_flu(
            body_pose(north),
            EnuVector(Vector3::zeros()),
            EnuVector(Vector3::new(0.0, 0.0, 1.0)),
        );
        assert_flu(about_up.angular(), [0.0, 0.0, 1.0]);
    }
}
