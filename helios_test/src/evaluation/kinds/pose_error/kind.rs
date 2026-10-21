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

    /// The truth is interpolated to each estimate's stamp.
    fn other_lining(&self) -> Lining<RootedPose> {
        Lining::Interpolate(blend_rooted_poses)
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

    fn bind(
        params: PoseErrorConfig,
        estimate: &BoundInput,
        truth: &BoundInput,
    ) -> Result<Self, Vec<String>> {
        let (estimated, true_frame) = match (estimate.frame(Self::BASE), truth.frame(Self::OTHER)) {
            (Ok(estimated), Ok(true_frame)) => (estimated, true_frame),
            (estimated, true_frame) => {
                return Err(estimated
                    .err()
                    .into_iter()
                    .chain(true_frame.err())
                    .collect())
            }
        };
        match (&estimated, &true_frame) {
            (Some(estimated), Some(true_frame)) if estimated == true_frame => Ok(Self {
                align: params.align,
            }),
            (estimated, true_frame) => Err(vec![format!(
                "the {} is the pose of '{}' and the {} of '{}': {} grades one frame, so select \
                 the same instance for both",
                Self::BASE,
                spelled(estimated.as_ref()),
                Self::OTHER,
                spelled(true_frame.as_ref()),
                Self::NAME,
            )]),
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

    use crate::evaluation::{derived::Refused, kind::EvaluatorKind, typed::TwoInputsKind};

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::transforms::{ErasedTransform, Transform};

    use nalgebra::{Isometry3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    const CAR: &str = "car";

    fn input(instance: Option<FrameId>) -> BoundInput {
        BoundInput {
            agent: AgentId::new(CAR),
            instance: instance.map(|frame| frame.to_string()),
        }
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
        );
        let no_instance = kind()
            .build(&toml::Table::new(), &[input(Some(base_link)), input(None)])
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
            agent: AgentId::new(CAR),
            instance: Some(spelling.to_owned()),
        };

        let problems = kind()
            .build(
                &toml::Table::new(),
                &[ros_frame("base_link"), ros_frame("odom")],
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
