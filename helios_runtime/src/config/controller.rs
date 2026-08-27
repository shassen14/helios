use crate::config::CommandSpace;

use serde::Deserialize;

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum ControllerStateSourceConfig {
    #[default]
    Estimated,
    GroundTruth,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum ControllerConfig {
    /// Passes the PathFollower reference's body-frame velocity through to a
    /// BodyTwist unchanged. Use this when a PathFollower (e.g., PurePursuit) has
    /// already computed velocity commands and no additional feedback is needed;
    /// whichever velocity DOF the reference carries pass through.
    DirectTwist {
        #[serde(default)]
        state_source: ControllerStateSourceConfig,
    },
    LongitudinalVelocity {
        #[serde(default)]
        state_source: ControllerStateSourceConfig,
        proportional_gain: f64,
        integral_gain: f64,
        derivative_gain: f64,
    },
    RoadLoad {
        c_roll: f64,
        c_drag: f64,
    },
    BicycleSteer {
        wheelbase: f64,
    },
}

impl ControllerConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            ControllerConfig::DirectTwist { .. } => "DirectTwist",
            ControllerConfig::LongitudinalVelocity { .. } => "LongitudinalVelocity",
            ControllerConfig::RoadLoad { .. } => "RoadLoad",
            ControllerConfig::BicycleSteer { .. } => "BicycleSteer",
        }
    }

    /// The command space this controller emits. Every controller feeding a given
    /// allocator must agree with the allocator's [`command_space`]; validation
    /// enforces it, since the DAG erases the type at the channel boundary.
    ///
    /// [`command_space`]: crate::config::AllocatorConfig::command_space
    pub(crate) fn command_space(&self) -> CommandSpace {
        match self {
            ControllerConfig::DirectTwist { .. } => CommandSpace::BodyTwist,
            ControllerConfig::LongitudinalVelocity { .. } => CommandSpace::DriveForce,
            ControllerConfig::RoadLoad { .. } => CommandSpace::DriveForce,
            ControllerConfig::BicycleSteer { .. } => CommandSpace::SteerAngle,
        }
    }

    /// Where this controller's contribution sits in the command fold: a feedback
    /// leg the fold requires fresh, or a feedforward leg it folds in when present.
    /// The assembler uses this to place each controller's channel into the summing
    /// node's `required` vs `optional` set — see [`FoldRole`].
    pub(crate) fn fold_role(&self) -> FoldRole {
        match self {
            // A passthrough of the reference twist; it never feeds a fold (its
            // command space is `BodyTwist`), so the value is moot — classed with
            // the feedback legs it resembles in reading the estimate.
            ControllerConfig::DirectTwist { .. } => FoldRole::Feedback,
            ControllerConfig::LongitudinalVelocity { .. } => FoldRole::Feedback,
            ControllerConfig::RoadLoad { .. } => FoldRole::Feedforward,
            ControllerConfig::BicycleSteer { .. } => FoldRole::Feedforward,
        }
    }

    pub fn state_source(&self) -> Option<ControllerStateSourceConfig> {
        match self {
            ControllerConfig::DirectTwist { state_source, .. } => Some(*state_source),
            ControllerConfig::LongitudinalVelocity { state_source, .. } => Some(*state_source),
            ControllerConfig::RoadLoad { .. } => None,
            ControllerConfig::BicycleSteer { .. } => None,
        }
    }
}

/// How a controller's output enters the command fold. Like [`CommandSpace`], a
/// derived dispatch tag computed from the controller kind, never parsed from TOML.
///
/// The two variants are exactly the summing node's two input classes:
/// - `Feedback` → a `required` input: presence-gated (a fold missing it publishes
///   nothing) and ordered by the topological sort, so it is read fresh this tick.
///   A correction that acts on a stale error is worse than none.
/// - `Feedforward` → an `optional` input: folded in when present, read
///   last-known-good, tolerant of being a tick old — fine for a slowly varying
///   open-loop term.
///
/// This is deliberately *not* inferred from [`state_source`](ControllerConfig::state_source):
/// "reads the estimate" and "must be fresh" are different properties. A
/// gravity-compensation feedforward reads state yet still folds as feedforward, so
/// the two must not be conflated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FoldRole {
    Feedback,
    Feedforward,
}
