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
    /// One source writes `command` itself — its producer node is retargeted onto
    /// the terminal, so no arbiter is synthesized. Both kinds qualify: autonomy's
    /// controller and teleop's mapper are alike internal producers.
    Direct(CommandSource),
    /// A synthesized `Selector` feeds `command` from two or more contending
    /// sources — the higher-priority ones freshness-gated `preferred` inputs, the
    /// lowest the always-available `base`.
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
    // command terminal otherwise. A lone source — of either kind — is retargeted
    // to write `command` directly, so it needs no arbiter.
    match arb.sources.as_slice() {
        [] if has_controller => CommandTopology::Direct(CommandSource::Autonomy),
        [] => CommandTopology::None,
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
    fn single_teleop_source_is_direct() {
        // Teleop's producer is the mapper node — an internal source like the
        // controller — so a lone teleop source is retargeted onto `command`
        // directly, no arbiter, symmetric with a lone autonomy source. The
        // assembler builds the mapper writing `command` for this topology.
        let topology = resolve_command_topology(&config(vec![CommandSource::Teleop]), false);
        assert!(matches!(
            topology,
            CommandTopology::Direct(CommandSource::Teleop)
        ));
    }

    #[test]
    fn single_autonomy_source_is_direct() {
        // A lone internal source: its producer is a DAG node retargeted onto
        // `command`, so no arbiter is synthesized — symmetric with a lone teleop
        // source now that the mapper makes teleop an internal producer too.
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
