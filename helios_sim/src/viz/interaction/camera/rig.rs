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

/// Bounds the rig clamps against, read by
/// [`apply_camera_intent`](super::intent::apply_camera_intent) on every write to
/// the rig. A `Resource`, not per-camera state: these are one operator preference
/// for the session, whereas [`CameraRig`] is per-camera view state. Defaults
/// reproduce the values that were compiled in before the tuning surface existed.
#[derive(Resource, Debug, Clone)]
pub struct CameraRigTuning {
    /// Angular margin (radians) held between the camera's pitch and straight
    /// up/down. At exactly vertical the view direction lines up with the `Vec3::Y`
    /// up-vector and `looking_at` cannot resolve roll; this margin keeps the rig
    /// clear of that singular pose.
    pub pitch_eps: f32,
    /// Closest the camera may orbit to its focus (meters). Zero or negative would
    /// place the camera on, or behind, the focus point.
    pub min_distance: f32,
    /// Farthest the camera may orbit from its focus (meters), kept inside the
    /// render far-plane so the scene never clips out.
    pub max_distance: f32,
}

impl Default for CameraRigTuning {
    fn default() -> Self {
        Self {
            pitch_eps: 0.05,
            min_distance: 2.0,
            max_distance: 500.0,
        }
    }
}

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

impl CameraRigTuning {
    /// Clamps a proposed pitch to just shy of vertical in both directions (by
    /// [`pitch_eps`](Self::pitch_eps)), keeping the view direction off the up-vector.
    ///
    /// Apply this on every *write* to [`CameraRig::pitch`], not at read time, so the
    /// stored rig never holds an out-of-range value — state and rendered view stay
    /// in agreement.
    pub(crate) fn clamp_pitch(&self, pitch: f32) -> f32 {
        let max_angle: f32 = FRAC_PI_2 - self.pitch_eps;
        pitch.clamp(-max_angle, max_angle)
    }

    /// Clamps a proposed orbit distance to `[min_distance, max_distance]` — near
    /// enough to stay outside the focus, far enough to stay inside the far-plane.
    pub(crate) fn clamp_distance(&self, distance: f32) -> f32 {
        distance.clamp(self.min_distance, self.max_distance)
    }
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

/// Maps a focus shift expressed in the camera's **ground frame** (`x` =
/// camera-right, `y` = camera-forward, both on the horizontal plane) into a
/// world-space `Vec3`, given the current `yaw`. Lets any input source express
/// "move the focus right/forward" without knowing the camera's heading; the
/// caller adds the result to [`CameraRig::focus`].
///
/// Sign-neutral by design: a source that wants "grab the world" pan negates its
/// own input. Shares [`rig_to_transform`]'s yaw convention — at `yaw` 0 the camera
/// looks down `-Z` with `+X` to its right — so pan and orbit never disagree.
pub(crate) fn ground_pan_to_world(ground: Vec2, yaw: f32) -> Vec3 {
    let right = Vec3::new(f32::cos(yaw), 0.0, -f32::sin(yaw));
    let forward = Vec3::new(-f32::sin(yaw), 0.0, -f32::cos(yaw));

    right * ground.x + forward * ground.y
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
        let tuning = CameraRigTuning::default();
        let limit = FRAC_PI_2 - tuning.pitch_eps;
        assert_eq!(tuning.clamp_pitch(2.0), limit); // past +vertical
        assert_eq!(tuning.clamp_pitch(-2.0), -limit); // past -vertical
        assert_eq!(tuning.clamp_pitch(0.3), 0.3); // in range, untouched
    }

    #[test]
    fn distance_saturates_to_its_bounds() {
        let tuning = CameraRigTuning::default();
        assert_eq!(tuning.clamp_distance(0.5), tuning.min_distance); // too close
        assert_eq!(tuning.clamp_distance(9000.0), tuning.max_distance); // too far
        assert_eq!(tuning.clamp_distance(50.0), 50.0); // in range, untouched
    }

    /// The clamps read their bounds off the tuning, not a compiled-in constant —
    /// a non-default tuning must move the saturation points. This is the whole
    /// reason the values became config, so it gets its own guard against a
    /// regression to a literal.
    #[test]
    fn clamps_track_non_default_bounds() {
        let tuning = CameraRigTuning {
            pitch_eps: 0.5,
            min_distance: 10.0,
            max_distance: 20.0,
        };
        assert_eq!(tuning.clamp_distance(5.0), 10.0); // clamped up to min
        assert_eq!(tuning.clamp_distance(50.0), 20.0); // clamped down to max
        assert_eq!(tuning.clamp_pitch(2.0), FRAC_PI_2 - 0.5); // margin follows pitch_eps
    }

    #[test]
    fn ground_pan_maps_right_and_forward_by_yaw() {
        // At yaw 0 the camera looks down -Z with +X to its right, so a pure
        // camera-right shift lands on +X and a pure forward shift on -Z.
        let right = ground_pan_to_world(Vec2::new(1.0, 0.0), 0.0);
        assert!(close(right, Vec3::X), "right at yaw 0 -> +X, got {right:?}");

        let forward = ground_pan_to_world(Vec2::new(0.0, 1.0), 0.0);
        assert!(
            close(forward, Vec3::new(0.0, 0.0, -1.0)),
            "forward at yaw 0 -> -Z, got {forward:?}"
        );

        // A quarter-turn of yaw rotates "right" onto the -Z axis.
        let turned = ground_pan_to_world(Vec2::new(1.0, 0.0), FRAC_PI_2);
        assert!(
            close(turned, Vec3::new(0.0, 0.0, -1.0)),
            "right at yaw pi/2 -> -Z, got {turned:?}"
        );
    }
}
