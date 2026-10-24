//! [`Nees`]: the kind, its inputs and the metrics it offers, and the checks
//! that bind one entry.

use super::{config::NeesConfig, grading::Grading};
use crate::evaluation::{
    kind::{BoundInput, InputRejection},
    kinds::pose_error::roots::DeclaredRoots,
    typed::{
        projections::{blend_rooted_poses, rooted_pose, RootedPose},
        Component, Lining, RowFacts, TwoInputs,
    },
};

use helios_core::estimation::consistency::{consistency, Consistency};
use helios_core::estimation::schema::StateSchema;
use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::{AgentId, FrameId};
use helios_core::spatial::state::Quantity;
use helios_core::spatial::transforms::tf::stamped::StampedTransform;
use helios_core::spatial::FrameAwareState;

/// The metric of every graded block's NEES jointly.
const JOINT: &str = "nees";

/// A state with a NaN or ±∞ in its mean or covariance.
const NOT_FINITE: InputRejection = InputRejection("a state that isn't finite");

/// A sample whose graded blocks' joint covariance has no NEES.
const JOINT_NOT_POSITIVE_DEFINITE: InputRejection =
    InputRejection("a state whose graded blocks' joint covariance isn't positive definite");

/// A state lacking a block bind graded. Its writer describes every state it
/// writes before the first, so this is reached only by a writer breaking
/// its description.
const OTHER_BLOCKS: InputRejection =
    InputRejection("a state without the blocks bind graded from its writer's description");

/// How a refusal of two unrelated roots closes for `nees`: no policy
/// relates them from a sample.
const NO_SAMPLE_ALIGNMENT: &str = "; nees aligns no roots from a sample, since that zeroes \
     the error its covariance is judged against";

/// _Per-step._ The `nees` kind, bound to one entry: grades whether a state's
/// covariance is honest about its error against a true pose of the body it
/// tracks. The estimate is a state channel whose writer describes its
/// schema and root; the truth is a transform edge whose child is that body.
/// Bind fixes the blocks graded, the body's position in the root and its
/// orientation, and relates the state's root to the truth's under
/// `declared`: one frame, or the root anchored at the truth's. Each sample
/// places the truth in the state's root, so each block's error is in the
/// tangent its covariance is kept in.
pub struct Nees {
    roots: DeclaredRoots,
    grading: Grading,
}

impl TwoInputs for Nees {
    const NAME: &'static str = "nees";
    /// The state an estimator writes, with its covariance.
    const BASE: &'static str = "estimate";
    /// The tracked body's true pose, from a truth source.
    const OTHER: &'static str = "truth";
    type Params = NeesConfig;
    type BasePayload = FrameAwareState;
    type BaseValue = FrameAwareState;
    type OtherPayload = StampedTransform;
    type OtherValue = RootedPose;
    type Fit = ();
    type State = ();
    type Output = Consistency;

    fn keep_base(&self, state: &FrameAwareState) -> Result<FrameAwareState, InputRejection> {
        if state.is_finite() {
            Ok(state.clone())
        } else {
            Err(NOT_FINITE)
        }
    }

    fn keep_other(&self, edge: &StampedTransform) -> Result<RootedPose, InputRejection> {
        rooted_pose(edge)
    }

    /// The truth is interpolated to each state's stamp, never across a gap
    /// wider than its cap.
    fn other_lining(&self) -> Lining<RootedPose> {
        Lining::InterpolateWithinCap(blend_rooted_poses)
    }

    /// One per graded block, known once the state's schema is bound.
    fn menu() -> Option<Vec<Component<Consistency>>> {
        None
    }

    fn check(params: &NeesConfig) -> Result<(), String> {
        params.refusal().map_or(Ok(()), Err)
    }

    /// The estimate must be a described state, its root related to the
    /// truth's under `declared`, holding a block a pose grades; the truth
    /// must be the pose of the body the state tracks.
    fn bind(
        _params: NeesConfig,
        estimate: &BoundInput,
        truth: &BoundInput,
    ) -> Result<Self, Vec<String>> {
        let roots = DeclaredRoots::bind::<Self>(estimate, truth, NO_SAMPLE_ALIGNMENT);
        let Some(schema) = estimate.schema.as_deref() else {
            let undescribed = format!(
                "the {}'s series carries no state its writer describes: {} reads a state \
                 channel whose schema the pipeline of agent '{}' gives",
                Self::BASE,
                Self::NAME,
                estimate.agent,
            );
            return Err(std::iter::once(undescribed)
                .chain(roots.err().into_iter().flatten())
                .collect());
        };
        let roots = roots?;
        let root = roots.estimate_root();
        let body = tracked_body(schema, root, &estimate.agent);

        let mut problems = Vec::new();
        match truth.frame(Self::OTHER) {
            Ok(Some(frame)) if frame == body => {}
            Ok(frame) => problems.push(format!(
                "the {} is the pose of '{}' and the {}'s state tracks '{body}': {} grades the \
                 tracked body, so select its instance for the {}",
                Self::OTHER,
                frame.map_or_else(|| "no instance".to_owned(), |frame| frame.to_string()),
                Self::BASE,
                Self::NAME,
                Self::OTHER,
            )),
            Err(problem) => problems.push(problem),
        }
        if let (Some(held), Some(owned)) = (schema.convention_of(root), estimate.convention) {
            if held != owned {
                problems.push(format!(
                    "the {}'s state holds its root '{root}' in {held} axes, but the root's \
                     owner gives {owned}: the estimator and the owner disagree",
                    Self::BASE,
                ));
            }
        }
        let grading = Grading::of(schema, &body, root);
        if grading.graded().is_empty() {
            problems.push(format!(
                "the {}'s state holds no block a true pose grades: neither {} nor {}",
                Self::BASE,
                Quantity::Position(root.clone()),
                Quantity::Orientation {
                    from: body.clone(),
                    to: root.clone(),
                },
            ));
        }
        if problems.is_empty() {
            Ok(Self { roots, grading })
        } else {
            Err(problems)
        }
    }

    /// `nees` over every graded block jointly, then each graded block's own,
    /// each dimensionless.
    fn components(&self) -> Vec<Component<Consistency>> {
        let joint = Component::each(JOINT, SiUnit::Dimensionless, |result: &Consistency| {
            result
                .joint()
                .map(|nees| nees.nees)
                .map_err(|_| JOINT_NOT_POSITIVE_DEFINITE)
        });
        let blocks = self.grading.graded().iter().enumerate().map(|(i, graded)| {
            let part = graded.part;
            Component::each(
                part.metric(),
                SiUnit::Dimensionless,
                move |result: &Consistency| {
                    result
                        .blocks()
                        .get(i)
                        .ok_or(OTHER_BLOCKS)?
                        .nees()
                        .map(|nees| nees.nees)
                        .map_err(|_| part.not_positive_definite())
                },
            )
        });
        std::iter::once(joint).chain(blocks).collect()
    }

    fn fit(&self, _run: &[(&FrameAwareState, &RootedPose)]) -> Result<(), InputRejection> {
        Ok(())
    }

    fn compute(
        &self,
        estimate: &FrameAwareState,
        truth: &RootedPose,
        _row: RowFacts,
        _state: &mut (),
        _fit: &(),
    ) -> Result<Consistency, InputRejection> {
        let truth = self.roots.truth_in_estimate_root(truth)?;
        let truths = self.grading.truths(&truth)?;
        consistency(estimate, &truths).map_err(|_| OTHER_BLOCKS)
    }

    /// How the roots relate, each graded block with its dof, and each
    /// ungraded block with why.
    fn binding(&self) -> toml::Table {
        std::iter::once(("roots".to_owned(), self.roots.to_string().into()))
            .chain(self.grading.binding())
            .collect()
    }
}

/// The body a state of `schema` held in `root` tracks: its orientation
/// block's child, or `agent`'s `base_link`, the body every estimator
/// tracks, when no orientation names it.
fn tracked_body(schema: &StateSchema, root: &FrameId, agent: &AgentId) -> FrameId {
    schema
        .blocks()
        .iter()
        .find_map(|block| match block.quantity() {
            Quantity::Orientation { from, to } if to == root => Some(from.clone()),
            _ => None,
        })
        .unwrap_or_else(|| FrameId::base_link(agent.clone()))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::{
        derived::Refused, kind::EvaluatorKind, kinds::pose_error::fixtures, typed::TwoInputsKind,
        Gap, GapWidth,
    };

    use helios_core::estimation::schema::StateSchemaBlock;
    use helios_core::kernel::manifold::TangentNoise;
    use helios_core::prelude::MonotonicTime;
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::transforms::{Convention, ErasedTransform, Transform};

    use nalgebra::{DMatrix, DVector, Isometry3, Translation3, UnitQuaternion};
    use std::sync::Arc;

    fn agent() -> AgentId {
        AgentId::new(fixtures::CAR)
    }

    fn base_link() -> FrameId {
        FrameId::base_link(agent())
    }

    fn odom() -> FrameId {
        FrameId::odom(agent())
    }

    fn noise() -> Option<TangentNoise> {
        TangentNoise::from_variances(DVector::from_element(3, 0.01))
    }

    fn flat(quantity: Quantity) -> StateSchemaBlock {
        StateSchemaBlock::new(
            quantity,
            Convention::Enu,
            noise(),
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        )
    }

    fn attitude() -> StateSchemaBlock {
        StateSchemaBlock::orientation(
            base_link(),
            odom(),
            Convention::Flu,
            Convention::Enu,
            noise(),
            DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
            DMatrix::identity(3, 3),
        )
    }

    /// The proving-ground EKF's blocks: position, velocity and orientation in
    /// odom, and a gyro bias on the body.
    fn ins() -> Arc<StateSchema> {
        Arc::new(StateSchema::compose(vec![
            flat(Quantity::Position(odom())),
            flat(Quantity::Velocity(odom())),
            attitude(),
            StateSchemaBlock::new(
                Quantity::GyroBias(base_link()),
                Convention::Flu,
                noise(),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
        ]))
    }

    fn position_only() -> Arc<StateSchema> {
        Arc::new(StateSchema::compose(vec![flat(Quantity::Position(odom()))]))
    }

    /// The state channel, built on `schema`, held in odom anchored at the
    /// datum.
    fn estimate(schema: Option<Arc<StateSchema>>) -> BoundInput {
        BoundInput {
            agent: agent(),
            instance: None,
            expressed_in: Some(odom()),
            convention: Some(Convention::Enu),
            placement: Some(fixtures::placement(
                odom(),
                Convention::Enu,
                Some(fixtures::datum()),
            )),
            schema,
            rate_hz: None,
            writer: None,
        }
    }

    /// `frame`'s true pose in the datum.
    fn truth(frame: Option<FrameId>) -> BoundInput {
        BoundInput {
            agent: agent(),
            instance: frame.map(|frame| frame.to_string()),
            expressed_in: Some(fixtures::datum()),
            convention: Some(Convention::Enu),
            placement: None,
            schema: None,
            rate_hz: None,
            writer: None,
        }
    }

    fn bound(estimate: BoundInput, truth: BoundInput) -> Result<Nees, Vec<String>> {
        Nees::bind(NeesConfig::default(), &estimate, &truth)
    }

    fn metrics(nees: &Nees) -> Vec<&'static str> {
        nees.components()
            .iter()
            .map(|component| component.metric().name)
            .collect()
    }

    /// base_link's true pose in the datum, at (`x`, 0, 0) facing east.
    fn true_pose(x: f64) -> RootedPose {
        let edge = StampedTransform {
            parent: fixtures::datum(),
            child: base_link(),
            stamp: MonotonicTime(0.0),
            transform: ErasedTransform::erase(Transform::<Flu, Enu>::from_isometry(
                Isometry3::from_parts(Translation3::new(x, 0.0, 0.0), UnitQuaternion::identity()),
            )),
        };
        rooted_pose(&edge).unwrap_or_else(|rejection| panic!("{rejection}"))
    }

    /// A state of `schema` with every block at its initial value and
    /// `covariance`.
    fn state(schema: Arc<StateSchema>, covariance: DMatrix<f64>) -> FrameAwareState {
        let mut state = FrameAwareState::from_schema(schema, MonotonicTime(0.0));
        state.covariance = covariance;
        state
    }

    fn row() -> RowFacts {
        RowFacts {
            at: MonotonicTime(0.0),
            follows_gap: false,
            other_fresh: None,
        }
    }

    /// Each metric's value at one sample, or the refusal counted.
    fn read(nees: &Nees, output: Consistency) -> Vec<(&'static str, Result<f64, Refused>)> {
        let sample = [(MonotonicTime(0.0), output)];
        nees.components()
            .iter()
            .map(|component| {
                let mut refused = Refused::default();
                let values = component.values(&sample, &mut refused);
                let value = match values.as_slice() {
                    [(_, value)] => Ok(*value),
                    _ => Err(refused),
                };
                (component.metric().name, value)
            })
            .collect()
    }

    #[test]
    fn an_ins_state_grades_position_and_orientation_and_names_the_rest() {
        let nees = bound(estimate(Some(ins())), truth(Some(base_link()))).expect("binds");

        assert_eq!(
            metrics(&nees),
            ["nees", "nees_position", "nees_orientation"]
        );
        let binding = nees.binding();
        assert_eq!(
            binding.get("roots").and_then(toml::Value::as_str),
            Some("agent.car.odom anchored at global.datum")
        );
        let graded: toml::Table = toml::from_str(
            "\"position in agent.car.odom\" = 3\n\
             \"orientation agent.car.base_link→agent.car.odom\" = 3",
        )
        .expect("valid TOML");
        let ungraded: toml::Table = toml::from_str(
            "\"velocity in agent.car.odom\" = \"a true pose gives no velocity\"\n\
             \"gyroscope bias in agent.car.base_link\" = \"no truth series gives it\"",
        )
        .expect("valid TOML");
        assert_eq!(binding.get("graded"), Some(&graded.into()));
        assert_eq!(binding.get("ungraded"), Some(&ungraded.into()));
    }

    #[test]
    fn a_position_only_state_offers_no_orientation_metric() {
        let nees = bound(estimate(Some(position_only())), truth(Some(base_link()))).expect("binds");

        assert_eq!(metrics(&nees), ["nees", "nees_position"]);
    }

    #[test]
    fn an_undescribed_state_is_refused_with_its_roots_problems() {
        let unrooted = BoundInput {
            expressed_in: None,
            ..estimate(None)
        };

        let problems = bound(unrooted, truth(Some(base_link())))
            .err()
            .expect("refused");

        assert_eq!(
            problems,
            [
                "the estimate's series carries no state its writer describes: nees reads a \
                 state channel whose schema the pipeline of agent 'car' gives",
                "the estimate's series declares no root its poses are expressed in",
            ]
        );
    }

    #[test]
    fn a_truth_of_another_frame_and_a_state_with_nothing_graded_are_refused() {
        let biases = Arc::new(StateSchema::compose(vec![flat(Quantity::AccelBias(
            base_link(),
        ))]));
        let wheel = FrameId::agent(agent(), "wheel");

        let problems = bound(estimate(Some(biases)), truth(Some(wheel)))
            .err()
            .expect("refused");

        assert_eq!(
            problems,
            [
                "the truth is the pose of 'agent.car.wheel' and the estimate's state tracks \
                 'agent.car.base_link': nees grades the tracked body, so select its instance \
                 for the truth",
                "the estimate's state holds no block a true pose grades: neither position in \
                 agent.car.odom nor orientation agent.car.base_link→agent.car.odom",
            ]
        );
    }

    #[test]
    fn roots_no_anchor_relates_are_refused_saying_nees_aligns_none_from_a_sample() {
        let unplaced = BoundInput {
            placement: None,
            ..estimate(Some(ins()))
        };

        let problems = bound(unplaced, truth(Some(base_link())))
            .err()
            .expect("refused");

        assert_eq!(
            problems,
            [
                "the estimate's root 'agent.car.odom' and the truth's root 'global.datum' are \
                 different frames, and align = \"declared\" grades poses in one, or through an \
                 anchor; nees aligns no roots from a sample, since that zeroes the error its \
                 covariance is judged against"
            ]
        );
    }

    #[test]
    fn first_pose_is_refused_at_load() {
        let kind = TwoInputsKind::<Nees>::new();
        let params: toml::Table = "align = \"first_pose\"".parse().expect("valid TOML");

        let problem = kind.check_params(&params).expect_err("refused");

        assert!(
            problem.starts_with("nees: align = \"first_pose\""),
            "{problem}"
        );
        assert_eq!(kind.menu(), None);
    }

    #[test]
    fn the_kind_binds_through_the_registry_adapter() {
        let kind = TwoInputsKind::<Nees>::new();
        let gap = Gap::Seconds(GapWidth::try_from(0.1).expect("positive and finite"));

        let evaluation = kind
            .build(
                &toml::Table::new(),
                &[estimate(Some(ins())), truth(Some(base_link()))],
                &[Some(gap), Some(gap)],
            )
            .unwrap_or_else(|problems| panic!("{problems:?}"));

        assert_eq!(evaluation.metrics().len(), 3);
    }

    #[test]
    fn each_sample_places_the_truth_in_the_root_and_grades_each_block() {
        let nees = bound(estimate(Some(ins())), truth(Some(base_link()))).expect("binds");
        let mut covariance = DMatrix::identity(12, 12);
        covariance[(0, 0)] = 4.0;
        let estimate = state(ins(), covariance);

        let output = nees
            .compute(&estimate, &true_pose(2.0), row(), &mut (), &())
            .expect("comparable");

        // The anchor places odom at the datum, so the truth is 2 m east of
        // the estimate, against a variance of 4 m²; the orientation is exact.
        assert_eq!(
            read(&nees, output),
            [
                ("nees", Ok(1.0)),
                ("nees_position", Ok(1.0)),
                ("nees_orientation", Ok(0.0)),
            ]
        );
    }

    #[test]
    fn a_zero_position_covariance_refuses_that_block_and_the_joint_alone() {
        let nees = bound(estimate(Some(ins())), truth(Some(base_link()))).expect("binds");
        let mut covariance = DMatrix::identity(12, 12);
        covariance.view_mut((0, 0), (3, 3)).fill(0.0);
        let estimate = state(ins(), covariance);

        let output = nees
            .compute(&estimate, &true_pose(0.0), row(), &mut (), &())
            .expect("comparable");

        let refused = |rejection| {
            Err(Refused {
                count: 1,
                first: Some(rejection),
            })
        };
        assert_eq!(
            read(&nees, output),
            [
                ("nees", refused(JOINT_NOT_POSITIVE_DEFINITE)),
                (
                    "nees_position",
                    refused(super::super::grading::Part::Position.not_positive_definite())
                ),
                ("nees_orientation", Ok(0.0)),
            ]
        );
    }

    #[test]
    fn a_state_that_isnt_finite_is_refused() {
        let nees = bound(estimate(Some(ins())), truth(Some(base_link()))).expect("binds");
        let mut broken = state(ins(), DMatrix::identity(12, 12));
        broken.mean[0] = f64::NAN;

        assert_eq!(nees.keep_base(&broken).err(), Some(NOT_FINITE));
    }
}
