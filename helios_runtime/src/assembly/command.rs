//! Command-seam assembly: names the synthesized fold nodes.
//!
//! Teleop-vs-autonomy arbitration happens one layer up, at the reference seam
//! (`seams/reference.rs`), so the command terminal itself is not arbitrated:
//! each command space is fed by its autonomy `Sum` fold.

use crate::config::CommandSpace;

/// Node name for the synthesized command fold. The dual of the reference
/// arbiter: where the arbiter *selects* one source, this *sums* a stack's
/// feedback and feedforward legs into the `command` terminal. Raw identity for
/// observability, so a referenced const; the assembler alone synthesizes it.
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

#[cfg(test)]
mod tests {
    use super::*;

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
