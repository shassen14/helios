//! [`FixedRoots`]: what `pose_error` carries over a run, how the estimate's
//! root sits in the truth's ([`Roots`]), fixed by the first sample, and each
//! estimate's difference from the truth at its stamp under it.

use super::config::Align;
use crate::evaluation::{kind::InputRejection, typed::projections::RootedPose};

use helios_core::prelude::FrameId;
use helios_core::spatial::transforms::{ErasedTransform, PoseDifference, RootAlignment};

/// Two poses tagged with different axis conventions, which no frame policy
/// relates. Bind refuses roots whose owners give them different conventions,
/// so this is reached only by a writer tagging its poses against its root's
/// owner.
const UNLIKE_CONVENTIONS: InputRejection =
    InputRejection("an estimate and a truth tagged with different axis conventions");

/// _Per-step._ What `pose_error` carries from sample to sample: the roots,
/// fixed by the first sample it computes, or why that sample couldn't fix
/// them, which refuses every later sample too. Fixed before that sample can
/// be refused, so a refused first sample never moves the alignment to the
/// second.
#[derive(Default)]
pub struct FixedRoots(Option<Result<Roots, InputRejection>>);

impl FixedRoots {
    /// `estimate` minus `truth` under the frame policy `align`, the roots
    /// fixed from this pair when it is the first, or why the two can't be
    /// compared.
    pub(super) fn difference(
        &mut self,
        align: Align,
        estimate: &RootedPose,
        truth: &RootedPose,
    ) -> Result<PoseDifference, InputRejection> {
        let roots = self
            .0
            .get_or_insert_with(|| Roots::first(align, estimate, truth));
        roots
            .as_ref()
            .map_err(|rejection| *rejection)?
            .difference(estimate, truth)
    }
}

/// _Per-step._ How the estimate's root sits in the truth's.
pub enum Roots {
    /// The two are one frame: bind checks the declared roots are, and the
    /// bus holds each writer to its declaration.
    Shared,
    /// The estimate's root `estimate`, placed in the truth's root `truth`
    /// by `alignment`.
    Aligned {
        estimate: FrameId,
        truth: FrameId,
        alignment: RootAlignment,
    },
}

impl Roots {
    /// The roots under the frame policy `align`, from the first sample's
    /// `estimate` and `truth`, or why they can't be fixed.
    fn first(
        align: Align,
        estimate: &RootedPose,
        truth: &RootedPose,
    ) -> Result<Self, InputRejection> {
        match align {
            Align::Declared => Ok(Roots::Shared),
            Align::FirstPose => Ok(Roots::Aligned {
                estimate: estimate.root.clone(),
                truth: truth.root.clone(),
                alignment: RootAlignment::from_pair(&estimate.pose, &truth.pose)
                    .map_err(|_| UNLIKE_CONVENTIONS)?,
            }),
            Align::BestFit | Align::Relative => {
                Err(InputRejection("a frame policy pose_error doesn't fit"))
            }
        }
    }

    /// `estimate` minus `truth`, the estimate placed in the truth's root, or
    /// why the two can't be compared.
    fn difference(
        &self,
        estimate: &RootedPose,
        truth: &RootedPose,
    ) -> Result<PoseDifference, InputRejection> {
        let estimate = self.place(estimate, truth)?;
        PoseDifference::between(&estimate, &truth.pose).map_err(|_| UNLIKE_CONVENTIONS)
    }

    /// `estimate`'s pose placed in `truth`'s root, or why it can't be. A
    /// root other than the one fitted is not reached from a run: bind
    /// compares the declared roots, and the bus refuses a value expressed in
    /// another frame than its writer declared.
    fn place(
        &self,
        estimate: &RootedPose,
        truth: &RootedPose,
    ) -> Result<ErasedTransform, InputRejection> {
        match self {
            Roots::Shared if estimate.root == truth.root => Ok(estimate.pose),
            Roots::Shared => Err(InputRejection(
                "an estimate and a truth in roots nothing relates; align = \"first_pose\" \
                 aligns them on their first sample, for an estimator that starts where its \
                 body is",
            )),
            Roots::Aligned {
                estimate: estimate_root,
                truth: truth_root,
                alignment,
            } if estimate.root == *estimate_root && truth.root == *truth_root => {
                Ok(alignment.apply(&estimate.pose))
            }
            Roots::Aligned { .. } => Err(InputRejection(
                "a pose whose root changed since the first sample, which aligned the two roots",
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::typed::projections::rooted_pose;

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::transforms::{
        tf::stamped::StampedTransform, ConventionOf, Transform,
    };

    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    const CAR: &str = "car";
    const TOL: f64 = 1e-12;

    fn base_link() -> FrameId {
        FrameId::base_link(AgentId::new(CAR))
    }

    fn odom() -> FrameId {
        FrameId::odom(AgentId::new(CAR))
    }

    fn datum() -> FrameId {
        FrameId::global("datum")
    }

    /// base_link's pose in `root`, at (`x`, `y`, 0) yawed `yaw`, kept. The
    /// edge is erased as its producer erases it: a `Transform` from the body
    /// to the root, whose convention is `Root`.
    fn kept<Root: ConventionOf>(root: FrameId, x: f64, y: f64, yaw: f64) -> RootedPose {
        let pose = Isometry3::from_parts(
            Translation3::new(x, y, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
        );
        let edge = StampedTransform {
            parent: root,
            child: base_link(),
            stamp: MonotonicTime(0.0),
            transform: ErasedTransform::erase(Transform::<Flu, Root>::from_isometry(pose)),
        };
        rooted_pose(&edge).unwrap_or_else(|rejection| panic!("{rejection}"))
    }

    /// Each sample's position, x, y, z and rotation under `align`, in
    /// stamp order, as the machinery computes them.
    fn grade(
        align: Align,
        run: &[(&RootedPose, &RootedPose)],
    ) -> Result<Vec<[f64; 5]>, InputRejection> {
        let mut roots = FixedRoots::default();
        run.iter()
            .map(|(estimate, truth)| {
                let difference = roots.difference(align, estimate, truth)?;
                let offset = difference.translation();
                Ok([
                    difference.position(),
                    offset.x,
                    offset.y,
                    offset.z,
                    difference.rotation(),
                ])
            })
            .collect()
    }

    fn close(actual: &[f64; 5], expected: &[f64; 5]) -> bool {
        actual
            .iter()
            .zip(expected)
            .all(|(actual, expected)| (actual - expected).abs() < TOL)
    }

    #[test]
    fn under_declared_poses_in_one_root_are_compared_directly() {
        let estimate = kept::<Enu>(datum(), 3.0, 4.0, 0.2);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

        let metrics = grade(Align::Declared, &[(&estimate, &truth)]).expect("one root");

        assert!(
            close(&metrics[0], &[5.0, 3.0, 4.0, 0.0, 0.2]),
            "{metrics:?}"
        );
    }

    #[test]
    fn under_declared_poses_in_two_roots_are_refused() {
        let estimate = kept::<Enu>(odom(), 0.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

        let refused = grade(Align::Declared, &[(&estimate, &truth)]).expect_err("two roots");

        assert!(refused.0.contains("align = \"first_pose\""), "{refused}");
    }

    #[test]
    fn under_first_pose_the_offset_between_roots_is_not_graded() {
        // Odom starts at the datum's (10, 0), turned a quarter turn: the
        // estimate and the truth agree at the first sample and again after
        // the car drives 2 m forward, and differ by 1 m to its left after.
        let first_estimate = kept::<Enu>(odom(), 0.0, 0.0, 0.0);
        let first_truth = kept::<Enu>(datum(), 10.0, 0.0, FRAC_PI_2);
        let driven = kept::<Enu>(odom(), 2.0, 0.0, 0.0);
        let driven_truth = kept::<Enu>(datum(), 10.0, 2.0, FRAC_PI_2);
        let drifted = kept::<Enu>(odom(), 2.0, 1.0, 0.0);

        let metrics = grade(
            Align::FirstPose,
            &[
                (&first_estimate, &first_truth),
                (&driven, &driven_truth),
                (&drifted, &driven_truth),
            ],
        )
        .expect("aligned");

        assert!(close(&metrics[0], &[0.0; 5]), "{metrics:?}");
        assert!(close(&metrics[1], &[0.0; 5]), "{metrics:?}");
        // The car's left is the datum's -x.
        assert!(
            close(&metrics[2], &[1.0, -1.0, 0.0, 0.0, 0.0]),
            "{metrics:?}"
        );
    }

    #[test]
    fn under_first_pose_a_root_that_changes_is_refused() {
        let first_estimate = kept::<Enu>(odom(), 0.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);
        let in_map = kept::<Enu>(FrameId::map(AgentId::new(CAR)), 0.0, 0.0, 0.0);

        let mut roots = FixedRoots::default();

        assert!(roots
            .difference(Align::FirstPose, &first_estimate, &truth)
            .is_ok());
        assert!(roots.difference(Align::FirstPose, &in_map, &truth).is_err());
    }

    #[test]
    fn under_first_pose_a_refused_first_sample_still_fixes_the_roots() {
        // The first pair can't be aligned; the second could, and is refused
        // all the same: the alignment is the first sample's, never moved on.
        let in_flu_root = kept::<Flu>(odom(), 0.0, 0.0, 0.0);
        let estimate = kept::<Enu>(odom(), 1.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);
        let mut roots = FixedRoots::default();

        let first = roots.difference(Align::FirstPose, &in_flu_root, &truth);
        let second = roots.difference(Align::FirstPose, &estimate, &truth);

        assert_eq!(first.err(), Some(UNLIKE_CONVENTIONS));
        assert_eq!(second.err(), Some(UNLIKE_CONVENTIONS));
    }

    #[test]
    fn poses_tagged_with_different_conventions_are_refused_under_either_policy() {
        for align in [Align::Declared, Align::FirstPose] {
            let in_flu_root = kept::<Flu>(datum(), 0.0, 0.0, 0.0);
            let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

            let refused = grade(align, &[(&in_flu_root, &truth)]);

            assert_eq!(refused, Err(UNLIKE_CONVENTIONS), "{align}");
        }
    }
}
