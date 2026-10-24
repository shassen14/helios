//! Which blocks of a state a true pose grades: [`Grading`], fixed at bind
//! from the state's schema, each graded block's true value taken from the
//! truth at every sample, and every other block named with why it is
//! ungraded.

use crate::evaluation::kind::InputRejection;

use helios_core::estimation::consistency::BlockTruth;
use helios_core::estimation::schema::StateSchema;
use helios_core::prelude::FrameId;
use helios_core::spatial::state::Quantity;
use helios_core::spatial::transforms::{Convention, ErasedTransform};

/// A true pose tagged with other body axes than the state's orientation
/// block. Bind checks the root's axes; only the truth's writer tags the
/// body's, so each sample is checked here.
const UNLIKE_BODY_AXES: InputRejection = InputRejection(
    "a true pose whose body axes aren't the ones the state's orientation is held in",
);

/// _Per-step._ The blocks of one state a pose of its tracked body in its
/// root grades, in schema order, and the rest with why not.
pub(super) struct Grading {
    graded: Vec<Graded>,
    ungraded: Vec<(Quantity, &'static str)>,
    /// The body's axes in the orientation block, when one is graded.
    body_axes: Option<Convention>,
}

impl Grading {
    /// The grading of `schema`'s blocks by a pose of `body` in `root`: its
    /// position in `root` and its `body → root` orientation.
    pub(super) fn of(schema: &StateSchema, body: &FrameId, root: &FrameId) -> Self {
        let mut graded = Vec::new();
        let mut ungraded = Vec::new();
        for block in schema.blocks() {
            let quantity = block.quantity();
            let part = match quantity {
                Quantity::Position(frame) if frame == root => Some(Part::Position),
                Quantity::Orientation { from, to } if from == body && to == root => {
                    Some(Part::Orientation)
                }
                _ => None,
            };
            let dof = schema.block_of(quantity).map(|block| block.tangent_dim());
            match (part, dof) {
                (Some(part), Some(dof)) => graded.push(Graded {
                    part,
                    quantity: quantity.clone(),
                    dof,
                }),
                _ => ungraded.push((quantity.clone(), why_ungraded(quantity))),
            }
        }
        let body_axes = graded
            .iter()
            .any(|graded| graded.part == Part::Orientation)
            .then(|| schema.convention_of(body))
            .flatten();
        Self {
            graded,
            ungraded,
            body_axes,
        }
    }

    /// Each graded block, in schema order.
    pub(super) fn graded(&self) -> &[Graded] {
        &self.graded
    }

    /// Each graded block's true value, in [`graded`](Self::graded)'s order,
    /// from `truth`, the body's true pose in the state's root; or why the
    /// pose can't give them.
    pub(super) fn truths(
        &self,
        truth: &ErasedTransform,
    ) -> Result<Vec<BlockTruth>, InputRejection> {
        let pose = truth.isometry();
        self.graded
            .iter()
            .map(|graded| match graded.part {
                Part::Position => Ok(BlockTruth::vector3(
                    graded.quantity.clone(),
                    pose.translation.vector,
                )),
                Part::Orientation if self.body_axes == Some(truth.from_convention()) => Ok(
                    BlockTruth::orientation(graded.quantity.clone(), pose.rotation),
                ),
                Part::Orientation => Err(UNLIKE_BODY_AXES),
            })
            .collect()
    }

    /// The report's `graded`, each graded block with its degrees of freedom,
    /// and `ungraded`, each other block with why, so χ² bounds can be read
    /// off the binding.
    pub(super) fn binding(&self) -> [(String, toml::Value); 2] {
        let graded = self
            .graded
            .iter()
            .map(|graded| {
                (
                    graded.quantity.to_string(),
                    toml::Value::Integer(graded.dof as i64),
                )
            })
            .collect::<toml::Table>();
        let ungraded = self
            .ungraded
            .iter()
            .map(|(quantity, why)| (quantity.to_string(), toml::Value::from(*why)))
            .collect::<toml::Table>();
        [
            ("graded".to_owned(), graded.into()),
            ("ungraded".to_owned(), ungraded.into()),
        ]
    }
}

/// _Per-step._ One graded block: which part of the pose gives it, its
/// quantity, and its degrees of freedom, the dof of its NEES.
pub(super) struct Graded {
    pub(super) part: Part,
    pub(super) quantity: Quantity,
    pub(super) dof: usize,
}

/// The part of a pose a graded block is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Part {
    Position,
    Orientation,
}

impl Part {
    /// The metric of this block's own NEES.
    pub(super) fn metric(self) -> &'static str {
        match self {
            Part::Position => "nees_position",
            Part::Orientation => "nees_orientation",
        }
    }

    /// A sample whose covariance of this block has no NEES.
    pub(super) fn not_positive_definite(self) -> InputRejection {
        match self {
            Part::Position => {
                InputRejection("a state whose position covariance isn't positive definite")
            }
            Part::Orientation => {
                InputRejection("a state whose orientation covariance isn't positive definite")
            }
        }
    }
}

/// Why a pose of the tracked body in the state's root can't grade a block
/// of `quantity`. Exhaustive, so a new quantity has to answer.
fn why_ungraded(quantity: &Quantity) -> &'static str {
    match quantity {
        Quantity::Position(_) | Quantity::Orientation { .. } => {
            "not the tracked body's pose in the state's root, which the truth gives"
        }
        Quantity::Velocity(_) => "a true pose gives no velocity",
        Quantity::Acceleration(_) => "a true pose gives no acceleration",
        Quantity::SpecificForce(_) => "a true pose gives no specific force",
        Quantity::AngularVelocity(_) => "a true pose gives no angular velocity",
        Quantity::AngularAcceleration(_) => "a true pose gives no angular acceleration",
        Quantity::Mag(_)
        | Quantity::MagBias(_)
        | Quantity::AccelBias(_)
        | Quantity::GyroBias(_) => "no truth series gives it",
    }
}
