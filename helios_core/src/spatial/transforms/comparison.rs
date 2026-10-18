//! Comparing two beliefs of one frame's pose: how far apart they are, and how
//! one tree's root sits in another's when the two poses of the frame relate
//! them. Each pose is the frame's pose in its root, tagged with conventions,
//! and two poses compared must be tagged alike.

use crate::spatial::transforms::{erased::FrameMismatch, Convention, ErasedTransform};

use nalgebra::{Isometry3, Vector3};

/// How short a forward axis levelled about up may be and still give a
/// heading: the sine of its angle from vertical. Only a pose facing up or
/// down to within rounding falls under it; one a hair off vertical still
/// has a heading, however sensitive.
const LEVEL_TOLERANCE: f64 = 1e-9;

/// How far an estimated pose of a frame is from its true pose, both in one
/// root: the estimate minus the truth. Read in the root's axes (`x`, `y`,
/// `z`), split about the root's up axis (`horizontal`, `vertical`,
/// `heading`), or along the true body's own axes (`along`, `cross`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseDifference {
    estimate: Isometry3<f64>,
    truth: Isometry3<f64>,
    /// The convention of the root both poses are in.
    root: Convention,
    /// The convention of the frame posed.
    body: Convention,
}

impl PoseDifference {
    /// The difference of `estimate` from `truth`, or a [`FrameMismatch`] when
    /// the two are tagged with different conventions: their numbers are on
    /// different axes.
    pub fn between(
        estimate: &ErasedTransform,
        truth: &ErasedTransform,
    ) -> Result<Self, FrameMismatch> {
        truth.same_conventions(estimate)?;
        Ok(Self {
            estimate: estimate.isometry(),
            truth: truth.isometry(),
            root: truth.to_convention(),
            body: truth.from_convention(),
        })
    }

    /// The estimated position minus the true one, on the root's axes.
    pub fn translation(&self) -> Vector3<f64> {
        self.estimate.translation.vector - self.truth.translation.vector
    }

    /// The distance between the two positions.
    pub fn position(&self) -> f64 {
        self.translation().norm()
    }

    /// The offset along the root's up axis: positive when the estimate is
    /// above the truth.
    pub fn vertical(&self) -> f64 {
        self.translation().dot(&self.root.up())
    }

    /// The distance between the two positions across the root's up axis.
    pub fn horizontal(&self) -> f64 {
        (self.translation() - self.vertical() * self.root.up()).norm()
    }

    /// The angle of the rotation from the true orientation to the estimated
    /// one, in `[0, π]`.
    pub fn rotation(&self) -> f64 {
        self.truth.rotation.angle_to(&self.estimate.rotation)
    }

    /// The offset along the way the true body faces: positive when the
    /// estimate is ahead of it. `None` for a frame with no forward axis.
    pub fn along(&self) -> Option<f64> {
        Some(self.on_true_body_axis(self.body.forward()?))
    }

    /// The offset across the way the true body faces: positive when the
    /// estimate is to its left. `None` for a frame with no forward axis.
    pub fn cross(&self) -> Option<f64> {
        Some(self.on_true_body_axis(self.body.left()?))
    }

    /// The estimated heading minus the true one about the root's up axis, in
    /// `(-π, π]`: positive when the estimate is turned left of the truth.
    /// Pitch and roll don't enter it. [`NoHeading`] when either pose has
    /// none.
    pub fn heading(&self) -> Result<f64, NoHeading> {
        let forward = self.body.forward().ok_or(NoHeading::NoForwardAxis)?;
        let up = self.root.up();
        let level = |pose: &Isometry3<f64>| {
            let facing = pose.rotation * forward;
            let levelled = facing - facing.dot(&up) * up;
            (levelled.norm() >= LEVEL_TOLERANCE)
                .then_some(levelled)
                .ok_or(NoHeading::FacingVertical)
        };
        let (estimate, truth) = (level(&self.estimate)?, level(&self.truth)?);
        Ok(truth.cross(&estimate).dot(&up).atan2(truth.dot(&estimate)))
    }

    /// The offset along `axis`, a direction on the posed frame's own axes,
    /// as the true pose orients it.
    fn on_true_body_axis(&self, axis: Vector3<f64>) -> f64 {
        self.translation().dot(&(self.truth.rotation * axis))
    }
}

/// Why a [`PoseDifference`] has no heading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoHeading {
    /// The frame posed is a world frame, which faces no way.
    NoForwardAxis,
    /// The estimate or the truth faces straight up or down, where no
    /// direction about up is the way it faces.
    FacingVertical,
}

/// Where one root sits in another: what carries a pose in the first (the
/// source) to the same pose in the second (the target).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RootAlignment {
    source_in_target: Isometry3<f64>,
}

impl RootAlignment {
    /// The alignment that makes one frame's pose `in_source`, in the source
    /// root, the same as its pose `in_target`, in the target root: the two
    /// agree on that pose and on everything rigidly placed from it. A
    /// [`FrameMismatch`] when the two are tagged with different conventions.
    pub fn from_pair(
        in_source: &ErasedTransform,
        in_target: &ErasedTransform,
    ) -> Result<Self, FrameMismatch> {
        in_target.same_conventions(in_source)?;
        Ok(Self {
            source_in_target: in_target.isometry() * in_source.isometry().inverse(),
        })
    }

    /// `pose`, a frame's pose in the source root, as a pose in the target
    /// root, tagged as `pose` is.
    pub fn apply(&self, pose: &ErasedTransform) -> ErasedTransform {
        ErasedTransform::from_parts(
            self.source_in_target * pose.isometry(),
            pose.from_convention(),
            pose.to_convention(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::spatial::{
        conventions::{Enu, Flu},
        transforms::Transform,
    };

    use nalgebra::{Translation3, UnitQuaternion};
    use std::f64::consts::FRAC_PI_2;

    const TOL: f64 = 1e-12;

    /// A body's pose at (`x`, `y`, 0) yawed `yaw` in an ENU root, erased
    /// as its producer erases it: a `Transform` from the body to the root.
    fn pose(x: f64, y: f64, yaw: f64) -> ErasedTransform {
        pose_3d(x, y, 0.0, yaw, 0.0)
    }

    /// A body's pose at (`x`, `y`, `z`) yawed `yaw` then pitched `pitch`
    /// about its own left axis, erased as [`pose`] is.
    fn pose_3d(x: f64, y: f64, z: f64, yaw: f64, pitch: f64) -> ErasedTransform {
        ErasedTransform::erase(Transform::<Flu, Enu>::from_isometry(isometry(
            x, y, z, yaw, pitch,
        )))
    }

    /// At (`x`, `y`, `z`), yawed `yaw` then pitched `pitch` about the
    /// yawed left axis.
    fn isometry(x: f64, y: f64, z: f64, yaw: f64, pitch: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(x, y, z),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw)
                * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pitch),
        )
    }

    #[test]
    fn the_difference_is_the_offset_on_the_roots_axes_and_the_angle_between() {
        let difference =
            PoseDifference::between(&pose(3.0, 4.0, 0.2), &pose(0.0, 0.0, 0.0)).expect("alike");

        assert!((difference.translation() - Vector3::new(3.0, 4.0, 0.0)).norm() < TOL);
        assert!((difference.position() - 5.0).abs() < TOL);
        assert!((difference.rotation() - 0.2).abs() < TOL);
    }

    #[test]
    fn the_offset_splits_about_the_roots_up_axis() {
        let difference =
            PoseDifference::between(&pose_3d(3.0, 4.0, -2.0, 0.0, 0.0), &pose(0.0, 0.0, 0.0))
                .expect("alike");

        assert!((difference.horizontal() - 5.0).abs() < TOL);
        assert!((difference.vertical() + 2.0).abs() < TOL);
    }

    #[test]
    fn along_and_cross_are_on_the_true_bodys_axes() {
        // The truth faces north (a quarter turn left of east): an estimate
        // 2 m north and 1 m west of it is 2 m ahead and 1 m to its left.
        let truth = pose(10.0, 0.0, FRAC_PI_2);
        let estimate = pose(9.0, 2.0, FRAC_PI_2);

        let difference = PoseDifference::between(&estimate, &truth).expect("alike");

        assert!(difference
            .along()
            .is_some_and(|along| (along - 2.0).abs() < TOL));
        assert!(difference
            .cross()
            .is_some_and(|cross| (cross - 1.0).abs() < TOL));
    }

    #[test]
    fn heading_is_signed_about_up_and_ignores_pitch() {
        let truth = pose_3d(0.0, 0.0, 0.0, 3.0, 0.0);
        let turned_left_and_pitched = pose_3d(0.0, 0.0, 0.0, 3.0 + 0.3, 0.4);
        let turned_right = pose_3d(0.0, 0.0, 0.0, 3.0 - 0.3, 0.0);
        // Across ±π: a heading error never reads near a full turn.
        let across = pose_3d(0.0, 0.0, 0.0, -3.0, 0.0);

        let heading = |estimate: &ErasedTransform| {
            PoseDifference::between(estimate, &truth)
                .expect("alike")
                .heading()
                .expect("a body frame")
        };

        assert!((heading(&turned_left_and_pitched) - 0.3).abs() < TOL);
        assert!((heading(&turned_right) + 0.3).abs() < TOL);
        assert!((heading(&across) - (2.0 * std::f64::consts::PI - 6.0)).abs() < TOL);
    }

    #[test]
    fn a_pose_facing_straight_up_or_down_has_no_heading() {
        let level = pose(0.0, 0.0, 0.0);
        let heading = |estimate: &ErasedTransform, truth: &ErasedTransform| {
            PoseDifference::between(estimate, truth)
                .expect("alike")
                .heading()
        };

        for pitch in [FRAC_PI_2, -FRAC_PI_2] {
            let vertical = pose_3d(0.0, 0.0, 0.0, 0.0, pitch);
            assert_eq!(heading(&level, &vertical), Err(NoHeading::FacingVertical));
            assert_eq!(heading(&vertical, &level), Err(NoHeading::FacingVertical));
        }
    }

    #[test]
    fn a_pose_just_short_of_vertical_still_has_a_heading() {
        // Pitched a microradian short of straight up, a quarter turn left.
        let truth = pose(0.0, 0.0, 0.0);
        let nearly_vertical = pose_3d(0.0, 0.0, 0.0, FRAC_PI_2, FRAC_PI_2 - 1e-6);

        let heading = PoseDifference::between(&nearly_vertical, &truth)
            .expect("alike")
            .heading()
            .expect("off vertical");

        assert!((heading - FRAC_PI_2).abs() < 1e-6, "{heading}");
    }

    #[test]
    fn a_frame_with_no_forward_axis_has_no_heading_along_or_cross() {
        let world_frame = |x: f64| {
            ErasedTransform::erase(Transform::<Enu, Enu>::from_isometry(
                Isometry3::translation(x, 0.0, 0.0),
            ))
        };

        let difference =
            PoseDifference::between(&world_frame(1.0), &world_frame(0.0)).expect("alike");

        assert_eq!(difference.heading(), Err(NoHeading::NoForwardAxis));
        assert_eq!(difference.along(), None);
        assert_eq!(difference.cross(), None);
        assert!((difference.position() - 1.0).abs() < TOL);
    }

    #[test]
    fn poses_tagged_with_different_conventions_are_not_compared_or_aligned() {
        let in_enu = pose(0.0, 0.0, 0.0);
        let in_flu = ErasedTransform::erase(Transform::<Flu, Flu>::identity());

        assert!(PoseDifference::between(&in_flu, &in_enu).is_err());
        assert!(RootAlignment::from_pair(&in_flu, &in_enu).is_err());
    }

    #[test]
    fn an_alignment_from_one_pair_carries_poses_rigidly_into_the_target_root() {
        // The source root starts at the target's (10, 0), a quarter turn
        // left: 2 m forward in the source is 2 m along the target's +y, and
        // 1 m to the left is 1 m along its -x.
        let alignment = RootAlignment::from_pair(&pose(0.0, 0.0, 0.0), &pose(10.0, 0.0, FRAC_PI_2))
            .expect("alike");

        let forward = alignment.apply(&pose(2.0, 0.0, 0.0));
        let left = alignment.apply(&pose(2.0, 1.0, 0.0));

        let at = |pose: &ErasedTransform| pose.isometry().translation.vector;
        assert!((at(&forward) - Vector3::new(10.0, 2.0, 0.0)).norm() < TOL);
        assert!((at(&left) - Vector3::new(9.0, 2.0, 0.0)).norm() < TOL);
        assert_eq!(left.to_convention(), Convention::Enu);
    }
}
