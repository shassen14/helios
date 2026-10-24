//! [`PoseError`]: the kind, its inputs and the metrics it offers, and the
//! checks that bind one entry.

use super::{
    config::{Align, PoseErrorConfig},
    roots::{roots_of, unlike_axes, DeclaredRoots, FixedRoots, Roots},
};
use crate::evaluation::{
    kind::{BoundInput, InputRejection},
    typed::{
        projections::{blend_rooted_poses, rooted_pose, RootedPose},
        Component, Lining, RowFacts, TwoInputs,
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
/// `align` parameter says how the two roots relate. Under `declared`, bind
/// relates them before any sample: one frame, or the estimate's root
/// anchored at the truth's, a GPS fix pinning an EKF's odom to the datum.
/// Under `first_pose`, the first sample's two poses place one in the other,
/// so a constant offset between the roots is not graded, nor the estimate's
/// error at that first sample. That fits an estimator with no anchor that
/// starts where its body is.
pub struct PoseError {
    roots: Roots,
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
    type Fit = ();
    type State = FixedRoots;
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

    /// Both inputs must be one frame's pose, and the roots, the frames the
    /// poses are expressed in, related under `align`.
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
        match roots(params.align, estimate, truth) {
            Ok(roots) if problems.is_empty() => Ok(Self { roots }),
            Ok(_) => Err(problems),
            Err(root_problems) => {
                problems.extend(root_problems);
                Err(problems)
            }
        }
    }

    fn components(&self) -> Vec<Component<PoseDifference>> {
        components()
    }

    fn fit(&self, _run: &[(&RootedPose, &RootedPose)]) -> Result<(), InputRejection> {
        Ok(())
    }

    /// Under `first_pose`, the first sample fixes how the roots sit.
    fn compute(
        &self,
        estimate: &RootedPose,
        truth: &RootedPose,
        _row: RowFacts,
        fixed: &mut FixedRoots,
        _fit: &(),
    ) -> Result<PoseDifference, InputRejection> {
        self.roots.difference(estimate, truth, fixed)
    }

    /// Under `declared`, how the roots relate, so one frame reads apart from
    /// an anchor.
    fn binding(&self) -> toml::Table {
        match &self.roots {
            Roots::Declared(declared) => {
                toml::Table::from_iter([("roots".to_owned(), declared.to_string().into())])
            }
            Roots::FirstPose => toml::Table::new(),
        }
    }
}

/// A bound input's frame as a refusal names it.
fn spelled(frame: Option<&FrameId>) -> String {
    frame.map_or_else(|| "no instance".to_owned(), FrameId::to_string)
}

/// How the roots of `estimate` and `truth` relate under `align`, or every
/// reason they can't. Under `first_pose` the two must be held in one
/// convention; under `declared`, as [`DeclaredRoots::bind`] relates them.
fn roots(align: Align, estimate: &BoundInput, truth: &BoundInput) -> Result<Roots, Vec<String>> {
    match align {
        Align::FirstPose => {
            let [(estimate_root, estimate_axes), (truth_root, truth_axes)] =
                roots_of::<PoseError>(estimate, truth)?;
            if estimate_axes == truth_axes {
                Ok(Roots::FirstPose)
            } else {
                Err(vec![unlike_axes::<PoseError>(
                    &estimate_root,
                    estimate_axes,
                    &truth_root,
                    truth_axes,
                )])
            }
        }
        Align::Declared => DeclaredRoots::bind::<PoseError>(
            estimate,
            truth,
            &format!(
                "; align = \"{}\" aligns them on their first sample, for an estimator that \
                 starts where its body is",
                Align::FirstPose
            ),
        )
        .map(Roots::Declared),
        Align::BestFit | Align::Relative => {
            Err(vec![format!("align = \"{align}\" is not built yet")])
        }
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

    use super::super::fixtures::{datum, placement, site, CAR};
    use crate::evaluation::{
        derived::Refused, kind::EvaluatorKind, typed::TwoInputsKind, Gap, GapWidth,
    };

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::transforms::{Convention, ErasedTransform, Transform};

    use nalgebra::{Isometry3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    /// Each role's gap, the truth's a cap it takes whatever its rate.
    fn gaps() -> [Option<Gap>; 2] {
        let seconds = GapWidth::try_from(0.1).expect("positive and finite");
        [Some(Gap::Seconds(seconds)); 2]
    }

    /// The pose of `instance` in the datum, held in ENU axes.
    fn input(instance: Option<FrameId>) -> BoundInput {
        rooted(instance, Some(datum()), Some(Convention::Enu))
    }

    /// The pose of `instance` in `root`, whose owner gives it `convention`,
    /// a frame no node defines.
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
            placement: None,
            schema: None,
            rate_hz: None,
            writer: None,
        }
    }

    /// base_link's pose in odom, held in `axes` and placed as the build
    /// places it: pinned through a GPS at `pinned_at`, or free.
    fn in_odom(axes: Convention, pinned_at: Option<FrameId>) -> BoundInput {
        BoundInput {
            placement: Some(placement(odom().expect("a frame"), axes, pinned_at)),
            ..rooted(base_link(), odom(), Some(axes))
        }
    }

    /// The `roots` the bound entry reports, or every reason it is refused.
    fn bound_roots(estimate: BoundInput) -> Result<Option<String>, Vec<String>> {
        kind()
            .build(
                &toml::Table::new(),
                &[estimate, input(base_link())],
                &gaps(),
            )
            .map(|evaluation| {
                evaluation
                    .binding()
                    .get("roots")
                    .and_then(|roots| roots.as_str().map(str::to_owned))
            })
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
    fn different_roots_no_node_places_are_refused_under_declared_naming_both() {
        let estimate = rooted(base_link(), odom(), Some(Convention::Enu));

        let problems = bound_roots(estimate).expect_err("declared roots that differ");

        assert_eq!(
            problems,
            [
                "the estimate's root 'agent.car.odom' and the truth's root 'global.datum' are \
                 different frames, and align = \"declared\" grades poses in one, or through \
                 an anchor; align = \"first_pose\" aligns them on their first sample, for an \
                 estimator that starts where its body is"
            ],
        );
    }

    #[test]
    fn one_root_under_declared_binds_and_reports_one_frame() {
        assert_eq!(
            bound_roots(input(base_link())),
            Ok(Some("global.datum, one frame".to_owned()))
        );
    }

    #[test]
    fn a_root_anchored_at_the_truths_binds_and_reports_the_anchor() {
        assert_eq!(
            bound_roots(in_odom(Convention::Enu, Some(datum()))),
            Ok(Some("agent.car.odom anchored at global.datum".to_owned()))
        );
    }

    #[test]
    fn an_anchor_relates_roots_held_in_different_conventions() {
        assert_eq!(
            bound_roots(in_odom(Convention::Flu, Some(datum()))),
            Ok(Some("agent.car.odom anchored at global.datum".to_owned()))
        );
    }

    #[test]
    fn an_anchor_placing_the_root_in_other_axes_than_its_owner_gives_is_refused() {
        let estimate = BoundInput {
            convention: Some(Convention::Enu),
            ..in_odom(Convention::Flu, Some(datum()))
        };

        let problems = bound_roots(estimate).expect_err("unlike anchor");

        assert_eq!(
            problems,
            [
                "the estimate's root 'agent.car.odom' is held in ENU axes, but its anchor at \
                 'global.datum' places it in FLU axes: the owner that gives its convention \
                 and the node that anchors it disagree"
            ],
        );
    }

    #[test]
    fn a_root_anchored_at_another_world_frame_is_refused_naming_both() {
        let problems = bound_roots(in_odom(Convention::Enu, Some(site()))).expect_err("elsewhere");

        assert_eq!(
            problems,
            [
                "the estimate's root 'agent.car.odom' is anchored at 'global.site', not at the \
                 truth's root 'global.datum': align = \"declared\" relates two roots only \
                 through an anchor at the truth's"
            ],
        );
    }

    #[test]
    fn a_free_root_is_refused_quoting_why_it_is_free() {
        let problems = bound_roots(in_odom(Convention::Enu, None)).expect_err("free");

        assert_eq!(
            problems,
            [
                "the estimate's root 'agent.car.odom' is not placed in 'global.datum': node \
                 \"estimator\" reads no input that pins the position of agent.car.odom"
            ],
        );
    }

    #[test]
    fn first_pose_reports_no_roots_and_ignores_the_placement() {
        let bound = kind().build(
            &params("align = \"first_pose\""),
            &[in_odom(Convention::Enu, None), input(base_link())],
            &gaps(),
        );

        let evaluation = bound.unwrap_or_else(|problems| panic!("{problems:?}"));
        assert!(evaluation.binding().is_empty());
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
