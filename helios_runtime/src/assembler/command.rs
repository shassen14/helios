//! Command-seam assembly: resolves the declared arbitration config into a
//! wiring topology for the `command` terminal, and maps declared sources and
//! policy onto their runtime channel / `Selector` equivalents.

use crate::channels::control;
use crate::config::{ArbitrationPolicyConfig, CommandArbitrationConfig, CommandSource};
use crate::nodes::combinators::SelectorPolicy;
use crate::port::InternalChannel;

use helios_core::control::commands::BodyTwist;

/// Node name for the synthesized command arbiter. Raw identity (observability
/// keys on it), so it is a referenced const rather than a constructor-hidden
/// literal — the assembler is its sole synthesizer.
pub(crate) const COMMAND_ARBITER_NODE: &str = "command_arbiter";

pub(crate) const TELEOP_MAPPER_NODE: &str = "teleop_mapper";

/// How the `command` terminal is fed, resolved from the declared source set.
pub(super) enum CommandTopology {
    /// Pure-perception agent: nothing produces a `command` terminal.
    None,
    /// One internal source writes `command` itself — its producer node is
    /// retargeted onto the terminal, so no arbiter is synthesized. Autonomy
    /// only; a lone host source cannot be retargeted and takes `Arbitrated`.
    Direct(CommandSource),
    /// A synthesized `Selector` feeds `command`. Covers two-or-more contending
    /// sources and the degenerate lone-host-source relay (`preferred` empty,
    /// `base` the single host channel, forwarded verbatim).
    Arbitrated {
        preferred: Vec<CommandSource>,
        base: CommandSource,
    },
}

/// Resolves the arbitration config into a topology. An empty source set infers
/// autonomy-only when a controller exists, and no command terminal otherwise.
pub(super) fn resolve_command_topology(
    arb: &CommandArbitrationConfig,
    has_controller: bool,
) -> CommandTopology {
    // Highest authority is first; the lowest-priority source is the always-
    // available `base` fallback and the rest are freshness-gated `preferred`.
    // No declared sources infers autonomy-only when a controller exists, and no
    // command terminal otherwise. A lone source forks on kind: an internal
    // source (autonomy) writes `command` directly; a host source (teleop) cannot
    // be retargeted, so it relays through a one-input `Selector` (`Arbitrated`
    // with an empty `preferred`).
    match arb.sources.as_slice() {
        [] if has_controller => CommandTopology::Direct(CommandSource::Autonomy),
        [] => CommandTopology::None,
        [one] if one.is_host_published() => CommandTopology::Arbitrated {
            preferred: Vec::new(),
            base: *one,
        },
        [one] => CommandTopology::Direct(*one),
        [preferred @ .., base] => CommandTopology::Arbitrated {
            preferred: preferred.to_vec(),
            base: *base,
        },
    }
}

/// Maps a declared command source to its role channel (`BodyTwist` until the
/// actuator seam).
pub(super) fn source_channel(source: CommandSource) -> InternalChannel {
    match source {
        CommandSource::Autonomy => control::autonomy::<BodyTwist>(),
        CommandSource::Teleop => control::teleop::<BodyTwist>(),
    }
}

/// Translates the configured arbitration policy into the runtime policy the
/// `Selector` applies.
pub(super) fn selector_policy(arb: &CommandArbitrationConfig) -> SelectorPolicy {
    match arb.policy {
        ArbitrationPolicyConfig::FreshnessOverride => SelectorPolicy::FreshnessOverride {
            max_age: arb.teleop_max_age_s,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(sources: Vec<CommandSource>) -> CommandArbitrationConfig {
        CommandArbitrationConfig {
            sources,
            ..Default::default()
        }
    }

    #[test]
    fn empty_sources_with_controller_infers_autonomy() {
        let topology = resolve_command_topology(&config(vec![]), true);
        assert!(matches!(
            topology,
            CommandTopology::Direct(CommandSource::Autonomy)
        ));
    }

    #[test]
    fn empty_sources_without_controller_is_none() {
        let topology = resolve_command_topology(&config(vec![]), false);
        assert!(matches!(topology, CommandTopology::None));
    }

    #[test]
    fn single_teleop_source_relays_through_selector() {
        // Teleop is host-published: it cannot be retargeted onto `command` the
        // way an internal controller is, so even alone it resolves to a one-input
        // `Selector` relay — `Arbitrated` with an empty `preferred` and teleop as
        // the sole `base`. A `Direct` here would leave the host writing `teleop`
        // while the allocator read an empty `command`, and the agent would not move.
        let topology = resolve_command_topology(&config(vec![CommandSource::Teleop]), false);
        match topology {
            CommandTopology::Arbitrated { preferred, base } => {
                assert!(preferred.is_empty());
                assert_eq!(base, CommandSource::Teleop);
            }
            _ => panic!("expected an Arbitrated relay for a lone teleop source"),
        }
    }

    #[test]
    fn single_autonomy_source_is_direct() {
        // The internal-source half of the lone-source fork: autonomy's producer
        // is a DAG node retargeted onto `command`, so no arbiter is synthesized.
        // This asymmetry with a lone teleop source is the whole point of
        // `CommandSource::is_host_published`.
        let topology = resolve_command_topology(&config(vec![CommandSource::Autonomy]), true);
        assert!(matches!(
            topology,
            CommandTopology::Direct(CommandSource::Autonomy)
        ));
    }

    #[test]
    fn two_sources_arbitrate_with_lowest_priority_as_base() {
        // Priority order is highest-first: teleop wins when fresh, autonomy is
        // the always-available fallback.
        let topology = resolve_command_topology(
            &config(vec![CommandSource::Teleop, CommandSource::Autonomy]),
            true,
        );
        match topology {
            CommandTopology::Arbitrated { preferred, base } => {
                assert_eq!(preferred, vec![CommandSource::Teleop]);
                assert_eq!(base, CommandSource::Autonomy);
            }
            _ => panic!("expected Arbitrated topology"),
        }
    }
}
