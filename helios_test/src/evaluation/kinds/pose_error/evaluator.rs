//! [`PoseErrorEvaluator`]: one `pose_error` entry's evaluation. Fits how
//! the estimate's root sits in the truth's, and gives each estimate's
//! difference from the truth at its stamp.

use super::config::Align;
use crate::evaluation::{
    kind::InputRejection,
    typed::{projections::RootedPose, Component, Input, TypedEvaluator},
};

use helios_core::prelude::FrameId;
use helios_core::spatial::transforms::{
    tf::stamped::StampedTransform, ErasedTransform, PoseDifference, RootAlignment,
};

/// Two poses tagged with different axis conventions, which no frame policy
/// relates.
const UNLIKE_CONVENTIONS: InputRejection =
    InputRejection("an estimate and a truth tagged with different axis conventions");

/// _Per-step._ One `pose_error` entry: grades one frame's pose under the
/// frame policy `align`. Which frame is the instance both inputs were bound
/// to, so every edge it is fed is that frame's.
pub struct PoseErrorEvaluator {
    align: Align,
}

impl PoseErrorEvaluator {
    pub(super) fn new(align: Align) -> Self {
        Self { align }
    }
}

impl TypedEvaluator for PoseErrorEvaluator {
    type Inputs = (
        Input<Self, StampedTransform, RootedPose>,
        Input<Self, StampedTransform, RootedPose>,
    );
    type Fit = Roots;
    type Output = PoseDifference;

    fn components(&self) -> Vec<Component<PoseDifference>> {
        super::kind::components()
    }

    fn fit(&self, run: &[(&RootedPose, &RootedPose)]) -> Result<Roots, InputRejection> {
        match self.align {
            Align::Declared => Ok(Roots::Shared),
            Align::FirstPose => {
                let Some((estimate, truth)) = run.first() else {
                    return Ok(Roots::Shared);
                };
                Ok(Roots::Aligned {
                    estimate: estimate.root.clone(),
                    truth: truth.root.clone(),
                    alignment: RootAlignment::from_pair(&estimate.pose, &truth.pose)
                        .map_err(|_| UNLIKE_CONVENTIONS)?,
                })
            }
            Align::BestFit | Align::Relative => {
                Err(InputRejection("a frame policy pose_error doesn't fit"))
            }
        }
    }

    fn compute(
        &self,
        (estimate, truth): (&RootedPose, &RootedPose),
        roots: &Roots,
    ) -> Result<PoseDifference, InputRejection> {
        let estimate = roots.place(estimate, truth)?;
        PoseDifference::between(&estimate, &truth.pose).map_err(|_| UNLIKE_CONVENTIONS)
    }
}

/// _Per-step._ What `pose_error` fits over a run: how the estimate's root
/// sits in the truth's.
pub enum Roots {
    /// The two are one frame, which each sample is checked for.
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
    /// `estimate`'s pose placed in `truth`'s root, or why it can't be.
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
    use helios_core::spatial::transforms::{ConventionOf, Transform};

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

    fn evaluator(align: Align) -> PoseErrorEvaluator {
        PoseErrorEvaluator::new(align)
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

    /// Each sample's position, x, y, z and rotation, the run fitted first,
    /// as the machinery does.
    fn grade(
        evaluator: &PoseErrorEvaluator,
        run: &[(&RootedPose, &RootedPose)],
    ) -> Result<Vec<[f64; 5]>, InputRejection> {
        let fit = evaluator.fit(run)?;
        run.iter()
            .map(|row| {
                let difference = evaluator.compute(*row, &fit)?;
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
        let evaluator = evaluator(Align::Declared);
        let estimate = kept::<Enu>(datum(), 3.0, 4.0, 0.2);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

        let metrics = grade(&evaluator, &[(&estimate, &truth)]).expect("one root");

        assert!(
            close(&metrics[0], &[5.0, 3.0, 4.0, 0.0, 0.2]),
            "{metrics:?}"
        );
    }

    #[test]
    fn under_declared_poses_in_two_roots_are_refused() {
        let evaluator = evaluator(Align::Declared);
        let estimate = kept::<Enu>(odom(), 0.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

        let refused = grade(&evaluator, &[(&estimate, &truth)]).expect_err("two roots");

        assert!(refused.0.contains("align = \"first_pose\""), "{refused}");
    }

    #[test]
    fn under_first_pose_the_offset_between_roots_is_not_graded() {
        // Odom starts at the datum's (10, 0), turned a quarter turn: the
        // estimate and the truth agree at the first sample and again after
        // the car drives 2 m forward, and differ by 1 m to its left after.
        let evaluator = evaluator(Align::FirstPose);
        let first_estimate = kept::<Enu>(odom(), 0.0, 0.0, 0.0);
        let first_truth = kept::<Enu>(datum(), 10.0, 0.0, FRAC_PI_2);
        let driven = kept::<Enu>(odom(), 2.0, 0.0, 0.0);
        let driven_truth = kept::<Enu>(datum(), 10.0, 2.0, FRAC_PI_2);
        let drifted = kept::<Enu>(odom(), 2.0, 1.0, 0.0);

        let metrics = grade(
            &evaluator,
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
        let evaluator = evaluator(Align::FirstPose);
        let first_estimate = kept::<Enu>(odom(), 0.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);
        let in_map = kept::<Enu>(FrameId::map(AgentId::new(CAR)), 0.0, 0.0, 0.0);
        let rows = [(&first_estimate, &truth), (&in_map, &truth)];

        let fit = evaluator.fit(&rows).expect("a first sample");

        assert!(evaluator.compute(rows[0], &fit).is_ok());
        assert!(evaluator.compute(rows[1], &fit).is_err());
    }

    #[test]
    fn poses_tagged_with_different_conventions_are_refused_under_either_policy() {
        for align in [Align::Declared, Align::FirstPose] {
            let evaluator = evaluator(align);
            let in_flu_root = kept::<Flu>(datum(), 0.0, 0.0, 0.0);
            let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

            let refused = grade(&evaluator, &[(&in_flu_root, &truth)]);

            assert_eq!(refused, Err(UNLIKE_CONVENTIONS), "{align}");
        }
    }
}
