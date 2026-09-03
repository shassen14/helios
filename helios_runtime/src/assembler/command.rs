//! Command-seam assembly: names the synthesized fold nodes and translates the
//! arbitration policy into the runtime `Selector` policy.
//!
//! Teleop-vs-autonomy arbitration happens one layer up, at the guidance
//! reference seam (see the reference roles in [`crate::channels::control`]), so
//! the command terminal itself is no longer arbitrated — each command space is
//! fed by its autonomy `Sum` fold. What remains here is the fold-node vocabulary
//! and the policy translation the reference arbiter reuses.

use crate::config::{ArbitrationPolicyConfig, CommandSpace, ReferenceArbitrationConfig};
use crate::nodes::combinators::SelectorPolicy;

/// Node name for the synthesized teleop mapper (`Intent → reference`). Raw
/// identity for observability, so a referenced const — the assembler alone
/// synthesizes it.
pub(crate) const TELEOP_MAPPER_NODE: &str = "teleop_mapper";

/// Node name for the synthesized reference arbiter: the `Selector` that resolves
/// the teleop and autonomy guidance references onto the single `reference` seam.
/// Raw identity (observability keys on it), so a referenced const rather than a
/// constructor-hidden literal — the assembler is its sole synthesizer.
pub(crate) const REFERENCE_ARBITER_NODE: &str = "reference_arbiter";

/// Node name for the synthesized command fold. The dual of the arbiter: where the
/// arbiter *selects* one source, this *sums* a stack's feedback and feedforward
/// legs into the `command` terminal. Raw identity for observability, so a
/// referenced const, like [`COMMAND_ARBITER_NODE`] — the assembler alone
/// synthesizes it.
pub(crate) const COMMAND_SUM_NODE: &str = "command_sum";

pub(crate) const COMMAND_SUM_DRIVE_FORCE_NODE: &str = "command_sum_drive_force";

pub(crate) const COMMAND_SUM_STEER_ANGLE_NODE: &str = "command_sum_steer_angle";

/// The [`COMMAND_SUM_NODE`] name specialized to one command space. A decoupled
/// stack folds each space its allocators consume into its own `Sum`, so the fold
/// nodes need distinct names or the DAG rejects the second as a duplicate. Like
/// [`source_channel`], the mapping and its literals live here, with the other
/// synthesized command-seam identities. `BodyTwist` never reaches a `Sum` — its
/// terminal is the arbiter path — so it maps to the base name as a total-match
/// fallback that no live wiring exercises.
pub(crate) fn command_sum_node_name(space: CommandSpace) -> &'static str {
    match space {
        CommandSpace::DriveForce => COMMAND_SUM_DRIVE_FORCE_NODE,
        CommandSpace::SteerAngle => COMMAND_SUM_STEER_ANGLE_NODE,
        CommandSpace::BodyTwist => COMMAND_SUM_NODE,
    }
}

/// Translates the configured arbitration policy into the runtime policy the
/// reference `Selector` applies.
pub(super) fn selector_policy(arb: &ReferenceArbitrationConfig) -> SelectorPolicy {
    match arb.policy {
        ArbitrationPolicyConfig::FreshnessOverride => SelectorPolicy::FreshnessOverride {
            max_age: arb.teleop_max_age_s,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn freshness_policy_carries_the_configured_max_age() {
        // The reference arbiter reuses this translation, so the configured
        // teleop max-age must reach the runtime `Selector` policy verbatim.
        let arb = ReferenceArbitrationConfig {
            teleop_max_age_s: 0.25,
            ..Default::default()
        };
        match selector_policy(&arb) {
            SelectorPolicy::FreshnessOverride { max_age } => {
                assert!((max_age - 0.25).abs() < f64::EPSILON);
            }
        }
    }

    #[test]
    fn command_sum_node_names_are_distinct_per_space() {
        // A decoupled stack folds each space separately; the fold nodes need
        // distinct names or the DAG rejects the second as a duplicate.
        assert_ne!(
            command_sum_node_name(CommandSpace::DriveForce),
            command_sum_node_name(CommandSpace::SteerAngle)
        );
    }
}
