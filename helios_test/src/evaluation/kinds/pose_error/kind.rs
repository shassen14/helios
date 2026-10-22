//! [`PoseError`]: the kind, its inputs and the metrics it offers, and the
//! checks that bind one entry.

use super::{
    config::{Align, PoseErrorConfig},
    roots::Roots,
};
use crate::evaluation::{
    kind::{BoundInput, InputRejection},
    typed::{
        projections::{blend_rooted_poses, rooted_pose, RootedPose},
        Component, Lining, TwoInputs,
    },
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::FrameId;
use helios_core::spatial::transforms::{tf::stamped::StampedTransform, NoHeading, PoseDifference};

/// A heading, along or cross of a frame that faces no way.
const FACES_NO_WAY: InputRejection =
    InputRejection("a frame with no forward axis, which has no heading, along or cross");

/// A heading of a pose facing straight up or down.
const FACES_STRAIGHT_UP: InputRejection =
    InputRejection("a pose facing straight up or down, which has no heading");

/// _Per-step._ The `pose_error` kind, bound to one entry: grades one frame's
/// pose under the frame policy `align`. Both inputs are edges of a transform
/// tree with one child, the frame graded; each edge's parent is its tree's
/// root. An edge's instance is its child, so binding both inputs to one
/// instance is what picks the frame, and no edge of another reaches it. Its
/// `align` parameter says how the two roots relate: one frame under
/// `declared`, or placed one in the other by the first sample's two poses
/// under `first_pose`, so a constant offset between the roots is not graded,
/// nor the estimate's error at that first sample. That fits an estimator that
/// starts where its body is. One that starts elsewhere and is pulled in by an
/// aiding sensor (an EKF at its origin until the first GPS fix) carries its
/// starting offset into every later sample.
pub struct PoseError {
    align: Align,
}

impl TwoInputs for PoseError {
    const NAME: &'static str = "pose_error";
    /// The frame's pose as an estimator believes it.
    const BASE: &'static str = "estimate";
    /// The frame's true pose, from a truth source.
    const OTHER: &'static str = "truth";
    type Params = PoseErrorConfig;
    type BasePayload = StampedTransform;
    type BaseValue = RootedPose;
    type OtherPayload = StampedTransform;
    type OtherValue = RootedPose;
    type Fit = Roots;
    type Output = PoseDifference;

    fn keep_base(&self, edge: &StampedTransform) -> Result<RootedPose, InputRejection> {
        rooted_pose(edge)
    }

    fn keep_other(&self, edge: &StampedTransform) -> Result<RootedPose, InputRejection> {
        rooted_pose(edge)
    }

    /// The truth is interpolated to each estimate's stamp, never across a
    /// gap wider than its cap.
    fn other_lining(&self) -> Lining<RootedPose> {
        Lining::InterpolateWithinCap(blend_rooted_poses)
    }

    fn menu() -> Option<Vec<Component<PoseDifference>>> {
        Some(components())
    }

    fn check(params: &PoseErrorConfig) -> Result<(), String> {
        match params.align.unbuilt_until() {
            None => Ok(()),
            Some(first_case) => Err(format!(
                "align = \"{}\" is not built yet; its first case will be {first_case}",
                params.align
            )),
        }
    }

    /// Both inputs must be one frame's pose, and both roots, the frames the
    /// poses are expressed in, held in one axis convention; under
    /// `declared`, the roots must be one frame.
    fn bind(
        params: PoseErrorConfig,
        estimate: &BoundInput,
        truth: &BoundInput,
    ) -> Result<Self, Vec<String>> {
        let mut problems = Vec::new();
        match (estimate.frame(Self::BASE), truth.frame(Self::OTHER)) {
            (Ok(Some(estimated)), Ok(Some(true_frame))) if estimated == true_frame => {}
            (Ok(estimated), Ok(true_frame)) => problems.push(format!(
                "the {} is the pose of '{}' and the {} of '{}': {} grades one frame, so select \
                 the same instance for both",
                Self::BASE,
                spelled(estimated.as_ref()),
                Self::OTHER,
                spelled(true_frame.as_ref()),
                Self::NAME,
            )),
            (estimated, true_frame) => {
                problems.extend(estimated.err().into_iter().chain(true_frame.err()))
            }
        }
        problems.extend(root_problems(params.align, estimate, truth));
        if problems.is_empty() {
            Ok(Self {
                align: params.align,
            })
        } else {
            Err(problems)
        }
    }

    fn components(&self) -> Vec<Component<PoseDifference>> {
        components()
    }

    fn fit(&self, run: &[(&RootedPose, &RootedPose)]) -> Result<Roots, InputRejection> {
        Roots::fit(self.align, run)
    }

    fn compute(
        &self,
        estimate: &RootedPose,
        truth: &RootedPose,
        roots: &Roots,
    ) -> Result<PoseDifference, InputRejection> {
        roots.difference(estimate, truth)
    }
}

/// A bound input's frame as a refusal names it.
fn spelled(frame: Option<&FrameId>) -> String {
    frame.map_or_else(|| "no instance".to_owned(), FrameId::to_string)
}

/// Why the roots of `estimate` and `truth` can't be compared under `align`,
/// if they can't: a root its writer didn't declare, one no owner gives a
/// convention, two held in different conventions, or, under `declared`, two
/// different frames.
fn root_problems(align: Align, estimate: &BoundInput, truth: &BoundInput) -> Vec<String> {
    let root_of = |role: &str, input: &BoundInput| match (&input.expressed_in, input.convention) {
        (Some(root), Some(convention)) => Ok((root.clone(), convention)),
        (Some(root), None) => Err(format!(
            "the {role}'s root '{root}' has no axis convention: no owner in the pipeline of \
             agent '{}' gives one",
            input.agent
        )),
        (None, _) => Err(format!(
            "the {role}'s series declares no root its poses are expressed in"
        )),
    };
    match (
        root_of(PoseError::BASE, estimate),
        root_of(PoseError::OTHER, truth),
    ) {
        (Ok((estimate_root, estimate_axes)), Ok((truth_root, truth_axes))) => {
            let mut problems = Vec::new();
            if align == Align::Declared && estimate_root != truth_root {
                problems.push(format!(
                    "the {}'s root '{estimate_root}' and the {}'s root '{truth_root}' are \
                     different frames, and align = \"{align}\" grades poses in one; \
                     align = \"{}\" aligns them on their first sample, for an estimator \
                     that starts where its body is",
                    PoseError::BASE,
                    PoseError::OTHER,
                    Align::FirstPose,
                ));
            }
            if estimate_axes != truth_axes {
                problems.push(format!(
                    "the {}'s root '{estimate_root}' is held in {estimate_axes} axes and the \
                     {}'s root '{truth_root}' in {truth_axes}: {} compares poses held in one \
                     axis convention",
                    PoseError::BASE,
                    PoseError::OTHER,
                    PoseError::NAME,
                ));
            }
            problems
        }
        (estimate_root, truth_root) => estimate_root
            .err()
            .into_iter()
            .chain(truth_root.err())
            .collect(),
    }
}

/// The estimate minus the truth, read every way a run file may grade it:
/// the distance between the two positions, split about the root's up axis,
/// on each of the root's axes, and along and across the way the true body
/// faces; the angle between the two orientations, and the heading error
/// about up.
fn components() -> Vec<Component<PoseDifference>> {
    vec![
        Component::each("position", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.position())
        }),
        Component::each("horizontal", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.horizontal())
        }),
        Component::each("vertical", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.vertical())
        }),
        Component::each("x", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.translation().x)
        }),
        Component::each("y", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.translation().y)
        }),
        Component::each("z", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.translation().z)
        }),
        Component::each("along", SiUnit::Meters, |d: &PoseDifference| {
            d.along().ok_or(FACES_NO_WAY)
        }),
        Component::each("cross", SiUnit::Meters, |d: &PoseDifference| {
            d.cross().ok_or(FACES_NO_WAY)
        }),
        Component::each("rotation", SiUnit::Radians, |d: &PoseDifference| {
            Ok(d.rotation())
        }),
        Component::each("heading", SiUnit::Radians, |d: &PoseDifference| {
            d.heading().map_err(|reason| match reason {
                NoHeading::NoForwardAxis => FACES_NO_WAY,
                NoHeading::FacingVertical => FACES_STRAIGHT_UP,
            })
        }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::{
        derived::Refused, kind::EvaluatorKind, typed::TwoInputsKind, Gap, GapWidth,
    };

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::transforms::{Convention, ErasedTransform, Transform};

    use nalgebra::{Isometry3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    const CAR: &str = "car";

    /// Each role's gap, the truth's a cap it takes whatever its rate.
    fn gaps() -> [Option<Gap>; 2] {
        let seconds = GapWidth::try_from(0.1).expect("positive and finite");
        [None, Some(Gap::Seconds(seconds))]
    }

    /// The pose of `instance` in the datum, held in ENU axes.
    fn input(instance: Option<FrameId>) -> BoundInput {
        rooted(
            instance,
            Some(FrameId::global("datum")),
            Some(Convention::Enu),
        )
    }

    /// The pose of `instance` in `root`, whose owner gives it `convention`.
    fn rooted(
        instance: Option<FrameId>,
        root: Option<FrameId>,
        convention: Option<Convention>,
    ) -> BoundInput {
        BoundInput {
            agent: AgentId::new(CAR),
            instance: instance.map(|frame| frame.to_string()),
            expressed_in: root,
            convention,
            rate_hz: None,
            writer: None,
        }
    }

    fn base_link() -> Option<FrameId> {
        Some(FrameId::base_link(AgentId::new(CAR)))
    }

    fn odom() -> Option<FrameId> {
        Some(FrameId::odom(AgentId::new(CAR)))
    }

    fn kind() -> TwoInputsKind<PoseError> {
        TwoInputsKind::new()
    }

    fn params(text: &str) -> toml::Table {
        text.parse().expect("valid TOML")
    }

    #[test]
    fn inputs_on_two_frames_or_without_an_instance_are_refused() {
        let base_link = FrameId::base_link(AgentId::new(CAR));
        let wheel = FrameId::agent(AgentId::new(CAR), "wheel");

        let two_frames = kind().build(
            &toml::Table::new(),
            &[input(Some(base_link.clone())), input(Some(wheel))],
            &gaps(),
        );
        let no_instance = kind()
            .build(
                &toml::Table::new(),
                &[input(Some(base_link)), input(None)],
                &gaps(),
            )
            .err()
            .expect("no truth instance");

        assert!(two_frames.is_err());
        assert!(
            no_instance[0].contains("select the same instance for both"),
            "{no_instance:?}"
        );
    }

    #[test]
    fn instances_that_arent_frames_are_refused_each_by_role_and_spelling() {
        let ros_frame = |spelling: &str| BoundInput {
            instance: Some(spelling.to_owned()),
            ..input(None)
        };

        let problems = kind()
            .build(
                &toml::Table::new(),
                &[ros_frame("base_link"), ros_frame("odom")],
                &gaps(),
            )
            .err()
            .expect("neither is a frame");

        assert_eq!(problems.len(), 2, "{problems:?}");
        assert!(
            problems[0].contains("the estimate's instance 'base_link' is not a frame"),
            "{problems:?}"
        );
        assert!(
            problems[1].contains("the truth's instance 'odom' is not a frame"),
            "{problems:?}"
        );
    }

    #[test]
    fn roots_held_in_one_convention_bind() {
        let estimate = rooted(base_link(), odom(), Some(Convention::Enu));

        let bound = kind().build(
            &params("align = \"first_pose\""),
            &[estimate, input(base_link())],
            &gaps(),
        );

        assert!(bound.is_ok(), "{:?}", bound.err());
    }

    #[test]
    fn roots_held_in_different_conventions_are_refused_naming_both() {
        let estimate = rooted(base_link(), odom(), Some(Convention::Flu));

        let problems = kind()
            .build(
                &params("align = \"first_pose\""),
                &[estimate, input(base_link())],
                &gaps(),
            )
            .err()
            .expect("unlike roots");

        assert_eq!(
            problems,
            [
                "the estimate's root 'agent.car.odom' is held in FLU axes and the truth's root \
                 'global.datum' in ENU: pose_error compares poses held in one axis convention"
            ],
        );
    }

    #[test]
    fn different_roots_under_declared_are_refused_naming_both() {
        let estimate = rooted(base_link(), odom(), Some(Convention::Enu));

        let problems = kind()
            .build(
                &toml::Table::new(),
                &[estimate, input(base_link())],
                &gaps(),
            )
            .err()
            .expect("declared roots that differ");

        assert_eq!(
            problems,
            [
                "the estimate's root 'agent.car.odom' and the truth's root 'global.datum' are \
                 different frames, and align = \"declared\" grades poses in one; align = \
                 \"first_pose\" aligns them on their first sample, for an estimator that \
                 starts where its body is"
            ],
        );
    }

    #[test]
    fn a_root_without_a_convention_or_undeclared_is_refused_naming_it() {
        let unowned = rooted(base_link(), odom(), None);
        let undeclared = rooted(base_link(), None, None);

        let problems = kind()
            .build(&toml::Table::new(), &[unowned, undeclared], &gaps())
            .err()
            .expect("neither root can be compared");

        assert_eq!(
            problems,
            [
                "the estimate's root 'agent.car.odom' has no axis convention: no owner in the \
                 pipeline of agent 'car' gives one",
                "the truth's series declares no root its poses are expressed in",
            ],
        );
    }

    #[test]
    fn a_frame_policy_not_built_is_refused_naming_its_first_case() {
        let problem = kind()
            .check_params(&params("align = \"best_fit\""))
            .expect_err("best_fit is not built");

        assert!(
            problem.starts_with("pose_error: align = \"best_fit\""),
            "{problem}"
        );
        assert!(
            problem.contains("the first hardware or SLAM run"),
            "{problem}"
        );
        assert!(kind()
            .check_params(&params("align = \"first_pose\""))
            .is_ok());
        assert!(kind().check_params(&params("gain = 2.0")).is_err());
    }

    #[test]
    fn the_menu_reads_the_pose_every_way_with_its_units() {
        let menu: Vec<(&str, SiUnit)> = kind()
            .menu()
            .expect("a fixed menu")
            .iter()
            .map(|metric| (metric.name, metric.si_unit))
            .collect();

        assert_eq!(
            menu,
            vec![
                ("position", SiUnit::Meters),
                ("horizontal", SiUnit::Meters),
                ("vertical", SiUnit::Meters),
                ("x", SiUnit::Meters),
                ("y", SiUnit::Meters),
                ("z", SiUnit::Meters),
                ("along", SiUnit::Meters),
                ("cross", SiUnit::Meters),
                ("rotation", SiUnit::Radians),
                ("heading", SiUnit::Radians),
            ]
        );
    }

    #[test]
    fn a_heading_facing_straight_up_is_refused_and_counted_while_the_position_is_given() {
        let body_in_root = |pitch: f64| {
            ErasedTransform::erase(Transform::<Flu, Enu>::from_isometry(Isometry3::from_parts(
                Default::default(),
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pitch),
            )))
        };
        let difference =
            PoseDifference::between(&body_in_root(-FRAC_PI_2), &body_in_root(0.0)).expect("alike");
        let sample = [(MonotonicTime(1.0), difference)];
        let read = |name: &str| {
            let mut refused = Refused::default();
            let component = components()
                .into_iter()
                .find(|component| component.metric().name == name)
                .expect("offered");
            let values = component.values(&sample, &mut refused);
            (values.len(), refused)
        };

        assert_eq!(
            read("heading"),
            (
                0,
                Refused {
                    count: 1,
                    first: Some(FACES_STRAIGHT_UP),
                }
            )
        );
        assert_eq!(read("position"), (1, Refused::default()));
    }
}
