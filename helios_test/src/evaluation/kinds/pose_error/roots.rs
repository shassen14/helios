//! How `pose_error` places the estimate's root in the truth's: [`Roots`],
//! related at bind under `declared` ([`DeclaredRoots`]), or fixed by the
//! first sample under `first_pose` ([`FixedRoots`]), and each estimate's
//! difference from the truth at its stamp under it.

use crate::evaluation::{kind::InputRejection, typed::projections::RootedPose};

use helios_core::prelude::FrameId;
use helios_core::spatial::transforms::{ErasedTransform, PoseDifference, RootAlignment};

use std::fmt;

/// Two poses tagged with different axis conventions, which no frame policy
/// relates. Bind refuses roots whose owners give them different conventions,
/// unless an anchor relates them, so this is reached only by a writer
/// tagging its poses against its root's owner.
const UNLIKE_CONVENTIONS: InputRejection =
    InputRejection("an estimate and a truth tagged with different axis conventions");

/// _Per-step._ How the estimate's root sits in the truth's, as bind fixed
/// it for one entry.
pub enum Roots {
    /// Related at bind, before any sample.
    Declared(DeclaredRoots),
    /// Placed one in the other by the first sample's two poses, carried in
    /// [`FixedRoots`].
    FirstPose,
}

impl Roots {
    /// `estimate` minus `truth`, the estimate placed in the truth's root, or
    /// why the two can't be compared. `fixed` carries the first sample's
    /// alignment under `first_pose`.
    pub(super) fn difference(
        &self,
        estimate: &RootedPose,
        truth: &RootedPose,
        fixed: &mut FixedRoots,
    ) -> Result<PoseDifference, InputRejection> {
        match self {
            Roots::Declared(declared) => declared.difference(estimate, truth),
            Roots::FirstPose => fixed.difference(estimate, truth),
        }
    }
}

/// _Per-step._ The two roots under `declared`: one frame, or the estimate's
/// anchored at the truth's. Never fixed from a sample.
pub struct DeclaredRoots {
    estimate: FrameId,
    truth: FrameId,
    relation: Relation,
}

impl DeclaredRoots {
    /// Both inputs' poses in `root`.
    pub(super) fn one_frame(root: FrameId) -> Self {
        Self {
            estimate: root.clone(),
            truth: root,
            relation: Relation::OneFrame,
        }
    }

    /// The estimate's poses in `estimate`, anchored at `truth`, the truth's
    /// root, by `anchor`: `estimate`'s pose in `truth`.
    pub(super) fn anchored(estimate: FrameId, truth: FrameId, anchor: ErasedTransform) -> Self {
        Self {
            estimate,
            truth,
            relation: Relation::Anchored(anchor),
        }
    }

    fn difference(
        &self,
        estimate: &RootedPose,
        truth: &RootedPose,
    ) -> Result<PoseDifference, InputRejection> {
        let estimate = self.place(estimate, truth)?;
        PoseDifference::between(&estimate, &truth.pose).map_err(|_| UNLIKE_CONVENTIONS)
    }

    /// `estimate`'s pose placed in `truth`'s root, or why it can't be. A
    /// root other than the ones bind related is not reached from a run: the
    /// bus refuses a value expressed in another frame than its writer
    /// declared.
    fn place(
        &self,
        estimate: &RootedPose,
        truth: &RootedPose,
    ) -> Result<ErasedTransform, InputRejection> {
        if estimate.root != self.estimate || truth.root != self.truth {
            return Err(InputRejection(
                "a pose in another root than its writer declared, which bind related",
            ));
        }
        match &self.relation {
            Relation::OneFrame => Ok(estimate.pose),
            Relation::Anchored(anchor)
                if estimate.pose.to_convention() == anchor.from_convention() =>
            {
                Ok(estimate.pose.then(*anchor))
            }
            Relation::Anchored(_) => Err(UNLIKE_CONVENTIONS),
        }
    }
}

/// How the report states the relation: `<root>, one frame` or `<estimate's
/// root> anchored at <truth's root>`.
impl fmt::Display for DeclaredRoots {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.relation {
            Relation::OneFrame => write!(f, "{}, one frame", self.truth),
            Relation::Anchored(_) => write!(f, "{} anchored at {}", self.estimate, self.truth),
        }
    }
}

/// How the estimate's root sits in the truth's under `declared`.
enum Relation {
    /// The two are one frame.
    OneFrame,
    /// The estimate's root, posed in the truth's by its anchor's edge.
    Anchored(ErasedTransform),
}

/// _Per-step._ What `pose_error` carries from sample to sample under
/// `first_pose`: the roots, aligned by the first sample it computes, or why
/// that sample couldn't align them, which refuses every later sample too.
/// Fixed before that sample can be refused, so a refused first sample never
/// moves the alignment to the second.
#[derive(Default)]
pub struct FixedRoots(Option<Result<Aligned, InputRejection>>);

impl FixedRoots {
    /// `estimate` minus `truth`, the roots aligned on this pair when it is
    /// the first, or why the two can't be compared.
    fn difference(
        &mut self,
        estimate: &RootedPose,
        truth: &RootedPose,
    ) -> Result<PoseDifference, InputRejection> {
        let aligned = self
            .0
            .get_or_insert_with(|| Aligned::first(estimate, truth));
        let estimate = aligned
            .as_ref()
            .map_err(|rejection| *rejection)?
            .place(estimate, truth)?;
        PoseDifference::between(&estimate, &truth.pose).map_err(|_| UNLIKE_CONVENTIONS)
    }
}

/// _Per-step._ The estimate's root `estimate`, placed in the truth's root
/// `truth` by `alignment`.
struct Aligned {
    estimate: FrameId,
    truth: FrameId,
    alignment: RootAlignment,
}

impl Aligned {
    /// The roots aligned on the first sample's `estimate` and `truth`, or
    /// why they can't be.
    fn first(estimate: &RootedPose, truth: &RootedPose) -> Result<Self, InputRejection> {
        Ok(Self {
            estimate: estimate.root.clone(),
            truth: truth.root.clone(),
            alignment: RootAlignment::from_pair(&estimate.pose, &truth.pose)
                .map_err(|_| UNLIKE_CONVENTIONS)?,
        })
    }

    /// `estimate`'s pose placed in `truth`'s root, or why it can't be.
    fn place(
        &self,
        estimate: &RootedPose,
        truth: &RootedPose,
    ) -> Result<ErasedTransform, InputRejection> {
        if estimate.root == self.estimate && truth.root == self.truth {
            Ok(self.alignment.apply(&estimate.pose))
        } else {
            Err(InputRejection(
                "a pose whose root changed since the first sample, which aligned the two roots",
            ))
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
        tf::stamped::StampedTransform, Convention, ConventionOf, EdgeEnds, Transform,
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

    /// Odom anchored at the datum `x` m along its x axis, odom held in
    /// `axes`.
    fn anchored_at(x: f64, axes: Convention) -> Roots {
        let anchor = ErasedTransform::new(
            Isometry3::translation(x, 0.0, 0.0),
            EdgeEnds {
                child: axes,
                parent: Convention::Enu,
            },
        );
        Roots::Declared(DeclaredRoots::anchored(odom(), datum(), anchor))
    }

    /// Each sample's position, x, y, z and rotation under `roots`, in
    /// stamp order, as the machinery computes them.
    fn grade(
        roots: &Roots,
        run: &[(&RootedPose, &RootedPose)],
    ) -> Result<Vec<[f64; 5]>, InputRejection> {
        let mut fixed = FixedRoots::default();
        run.iter()
            .map(|(estimate, truth)| {
                let difference = roots.difference(estimate, truth, &mut fixed)?;
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
        let roots = Roots::Declared(DeclaredRoots::one_frame(datum()));
        let estimate = kept::<Enu>(datum(), 3.0, 4.0, 0.2);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

        let metrics = grade(&roots, &[(&estimate, &truth)]).expect("one root");

        assert!(
            close(&metrics[0], &[5.0, 3.0, 4.0, 0.0, 0.2]),
            "{metrics:?}"
        );
    }

    #[test]
    fn under_declared_a_pose_in_a_root_bind_did_not_relate_is_refused() {
        let roots = Roots::Declared(DeclaredRoots::one_frame(datum()));
        let estimate = kept::<Enu>(odom(), 0.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

        let refused = grade(&roots, &[(&estimate, &truth)]).expect_err("another root");

        assert!(refused.0.contains("another root"), "{refused}");
    }

    #[test]
    fn under_declared_an_anchored_estimate_is_placed_in_the_truths_root_by_its_anchor() {
        // Odom sits 10 m along the datum's x: the estimate 1 m along odom's
        // x is 11 m along the datum's, where the truth is, and a further
        // 1 m to the north is a 1 m error.
        let roots = anchored_at(10.0, Convention::Enu);
        let estimate = kept::<Enu>(odom(), 1.0, 0.0, 0.0);
        let drifted = kept::<Enu>(odom(), 1.0, 1.0, 0.0);
        let truth = kept::<Enu>(datum(), 11.0, 0.0, 0.0);

        let metrics = grade(&roots, &[(&estimate, &truth), (&drifted, &truth)]).expect("anchored");

        assert!(close(&metrics[0], &[0.0; 5]), "{metrics:?}");
        assert!(
            close(&metrics[1], &[1.0, 0.0, 1.0, 0.0, 0.0]),
            "{metrics:?}"
        );
    }

    #[test]
    fn under_declared_an_anchor_relates_roots_held_in_different_conventions() {
        let roots = anchored_at(0.0, Convention::Flu);
        let estimate = kept::<Flu>(odom(), 2.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 2.0, 0.0, 0.0);

        let metrics = grade(&roots, &[(&estimate, &truth)]).expect("related by the anchor");

        assert!(close(&metrics[0], &[0.0; 5]), "{metrics:?}");
    }

    #[test]
    fn under_declared_a_pose_tagged_against_its_anchor_is_refused() {
        let roots = anchored_at(0.0, Convention::Flu);
        let estimate = kept::<Enu>(odom(), 0.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

        let refused = grade(&roots, &[(&estimate, &truth)]);

        assert_eq!(refused, Err(UNLIKE_CONVENTIONS));
    }

    #[test]
    fn the_report_states_one_frame_or_the_anchor() {
        assert_eq!(
            DeclaredRoots::one_frame(datum()).to_string(),
            "global.datum, one frame"
        );
        let Roots::Declared(anchored) = anchored_at(0.0, Convention::Enu) else {
            unreachable!("anchored_at relates the roots at bind")
        };
        assert_eq!(
            anchored.to_string(),
            "agent.car.odom anchored at global.datum"
        );
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
            &Roots::FirstPose,
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

        let mut fixed = FixedRoots::default();

        assert!(Roots::FirstPose
            .difference(&first_estimate, &truth, &mut fixed)
            .is_ok());
        assert!(Roots::FirstPose
            .difference(&in_map, &truth, &mut fixed)
            .is_err());
    }

    #[test]
    fn under_first_pose_a_refused_first_sample_still_fixes_the_roots() {
        // The first pair can't be aligned; the second could, and is refused
        // all the same: the alignment is the first sample's, never moved on.
        let in_flu_root = kept::<Flu>(odom(), 0.0, 0.0, 0.0);
        let estimate = kept::<Enu>(odom(), 1.0, 0.0, 0.0);
        let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);
        let mut fixed = FixedRoots::default();

        let first = Roots::FirstPose.difference(&in_flu_root, &truth, &mut fixed);
        let second = Roots::FirstPose.difference(&estimate, &truth, &mut fixed);

        assert_eq!(first.err(), Some(UNLIKE_CONVENTIONS));
        assert_eq!(second.err(), Some(UNLIKE_CONVENTIONS));
    }

    #[test]
    fn poses_tagged_with_different_conventions_in_one_root_are_refused_under_either_policy() {
        for roots in [
            Roots::Declared(DeclaredRoots::one_frame(datum())),
            Roots::FirstPose,
        ] {
            let in_flu_root = kept::<Flu>(datum(), 0.0, 0.0, 0.0);
            let truth = kept::<Enu>(datum(), 0.0, 0.0, 0.0);

            let refused = grade(&roots, &[(&in_flu_root, &truth)]);

            assert_eq!(refused, Err(UNLIKE_CONVENTIONS));
        }
    }
}
