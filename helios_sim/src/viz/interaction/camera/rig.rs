//! The orbit-camera rig: a small piece of view state (`focus`, `distance`,
//! `yaw`, `pitch`) plus the pure spherical→Cartesian math that turns it into a
//! Bevy `Transform`.
//!
//! The rig lives *entirely* in Bevy Y-up space. It reads no bus data and
//! produces none — it only positions the camera — so nothing here crosses the
//! ENU↔Bevy boundary and no `core/transforms/` conversion belongs in this file.
//! That is the deliberate inverse of the `live/` gizmos, whose whole job is that
//! crossing.

use bevy::prelude::*;
use std::f32::consts::FRAC_PI_2;

/// Angular margin (radians) held between the camera's pitch and straight
/// up/down. At exactly vertical the view direction lines up with the `Vec3::Y`
/// up-vector and `looking_at` has no way to resolve roll; this margin keeps the
/// rig clear of that singular pose.
// TODO: pull from the viz config surface once it exists.
const PITCH_EPS: f32 = 0.05;

/// Closest the camera may orbit to its focus (meters). Zero or negative would
/// place the camera on, or behind, the focus point.
// TODO: pull from the viz config surface.
const MIN_DISTANCE: f32 = 2.0;

/// Farthest the camera may orbit from its focus (meters), kept inside the render
/// far-plane so the scene never clips out.
// TODO: pull from the viz config surface.
const MAX_DISTANCE: f32 = 500.0;

/// Orbit-camera state: the point the camera looks at (`focus`) and its position
/// as an offset from that point in spherical coordinates — `distance` out, `yaw`
/// swept around the vertical axis, `pitch` as elevation above the ground plane.
///
/// A `Component`, not a resource, because view state is per-camera: the
/// interactive systems mutate these fields and `rig_to_transform` reads them
/// each frame onto the camera's `Transform`.
#[derive(Component)]
pub struct CameraRig {
    /// World-space point the camera orbits and aims at (Bevy Y-up).
    pub(crate) focus: Vec3,
    /// Distance from `focus`, in meters.
    pub(crate) distance: f32,
    /// Rotation about the vertical (Y) axis, in radians.
    pub(crate) yaw: f32,
    /// Elevation above the horizontal (XZ) plane, in radians. Positive lifts the
    /// camera up and over the focus.
    pub(crate) pitch: f32,
}

/// Turns the rig's spherical state into a world `Transform`, positioned
/// `distance` from `focus` and aimed back at it.
///
/// Bevy is Y-up, so elevation (`pitch`) lands on the Y axis while `yaw` sweeps
/// the XZ ground plane — this is why the math differs from the physics/ENU
/// convention where Z is up. Pure and side-effect-free: this is the single
/// testable core of the camera.
pub(crate) fn rig_to_transform(rig: &CameraRig) -> Transform {
    let horizontal = rig.distance * f32::cos(rig.pitch);
    let x = horizontal * f32::sin(rig.yaw);
    let z = horizontal * f32::cos(rig.yaw);
    let y = rig.distance * f32::sin(rig.pitch);

    let camera_position = rig.focus + Vec3::new(x, y, z);

    Transform::from_translation(camera_position).looking_at(rig.focus, Vec3::Y)
}

/// Clamps a proposed pitch to just shy of vertical in both directions (see
/// [`PITCH_EPS`]), keeping the view direction off the up-vector.
///
/// Apply this on every *write* to [`CameraRig::pitch`], not at read time, so the
/// stored rig never holds an out-of-range value — state and rendered view stay
/// in agreement.
pub(crate) fn clamp_pitch(pitch: f32) -> f32 {
    let max_angle: f32 = FRAC_PI_2 - PITCH_EPS;

    pitch.clamp(-max_angle, max_angle)
}

/// Clamps a proposed orbit distance to `[MIN_DISTANCE, MAX_DISTANCE]` — near
/// enough to stay outside the focus, far enough to stay inside the far-plane.
pub(crate) fn clamp_distance(distance: f32) -> f32 {
    distance.clamp(MIN_DISTANCE, MAX_DISTANCE)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Loose Vec3 equality — trig round-trips and the rounded seed values below
    /// don't land on exact floats.
    fn close(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 0.1
    }

    #[test]
    fn zero_angles_place_the_camera_on_plus_z() {
        let rig = CameraRig {
            focus: Vec3::ZERO,
            distance: 10.0,
            yaw: 0.0,
            pitch: 0.0,
        };
        let t = rig_to_transform(&rig);
        assert!(
            close(t.translation, Vec3::new(0.0, 0.0, 10.0)),
            "got {:?}",
            t.translation
        );
    }

    #[test]
    fn seed_vantage_reproduces_the_old_fixed_camera() {
        // The spawn seed: distance/pitch/yaw chosen so the rig lands on the
        // previous hard-coded camera position (-30, 25, 30). If this drifts, the
        // window will open on a different view than it used to.
        let rig = CameraRig {
            focus: Vec3::ZERO,
            distance: 49.24,
            yaw: -0.785,
            pitch: 0.532,
        };
        let t = rig_to_transform(&rig);
        assert!(
            close(t.translation, Vec3::new(-30.0, 25.0, 30.0)),
            "got {:?}",
            t.translation
        );
    }

    #[test]
    fn camera_sits_at_distance_from_focus_and_looks_at_it() {
        let focus = Vec3::new(5.0, 1.0, -2.0);
        let rig = CameraRig {
            focus,
            distance: 20.0,
            yaw: 0.9,
            pitch: 0.4,
        };
        let t = rig_to_transform(&rig);

        // The orbit radius equals `distance` regardless of the angles.
        assert!(((t.translation - focus).length() - 20.0).abs() < 1e-2);

        // And the camera faces straight at the focus.
        let to_focus = (focus - t.translation).normalize();
        assert!(
            close(t.forward().as_vec3(), to_focus),
            "forward {:?} vs {:?}",
            t.forward(),
            to_focus
        );
    }

    #[test]
    fn pitch_saturates_just_below_vertical() {
        let limit = FRAC_PI_2 - PITCH_EPS;
        assert_eq!(clamp_pitch(2.0), limit); // past +vertical
        assert_eq!(clamp_pitch(-2.0), -limit); // past -vertical
        assert_eq!(clamp_pitch(0.3), 0.3); // in range, untouched
    }

    #[test]
    fn distance_saturates_to_its_bounds() {
        assert_eq!(clamp_distance(0.5), MIN_DISTANCE); // too close
        assert_eq!(clamp_distance(9000.0), MAX_DISTANCE); // too far
        assert_eq!(clamp_distance(50.0), 50.0); // in range, untouched
    }
}
