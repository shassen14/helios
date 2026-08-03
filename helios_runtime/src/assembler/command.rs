//! Command-seam assembly: resolves the declared arbitration config into a
//! wiring topology for the `command` terminal, and maps declared sources and
//! policy onto their runtime channel / `Selector` equivalents.

use crate::channels::control;
use crate::config::{ArbitrationPolicyConfig, CommandArbitrationConfig, CommandSource};
use crate::nodes::combinators::SelectorPolicy;
use crate::port::InternalChannel;

use helios_core::control::commands::BodyTwist;

/// How the `command` terminal is fed, resolved from the declared source set.
pub(super) enum CommandTopology {
    None,                  // pure-perception agent: no command terminal
    Direct(CommandSource), // 1 source writes `command` itself
    Arbitrated {
        preferred: Vec<CommandSource>,
        base: CommandSource,
    }, // 2 +
}

/// Resolves the arbitration config into a topology. An empty source set infers
/// autonomy-only when a controller exists, and no command terminal otherwise.
pub(super) fn resolve_command_topology(
    arb: &CommandArbitrationConfig,
    has_controller: bool,
) -> CommandTopology {
    // Highest authority is first; the lowest-priority source is the always-
    // available `base` fallback and the rest are freshness-gated `preferred`.
    // No declared sources infers autonomy-only when a controller exists, and
    // no command terminal otherwise.
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
        // Teleop-only: no controller required, teleop writes `command` itself.
        let topology = resolve_command_topology(&config(vec![CommandSource::Teleop]), false);
        assert!(matches!(
            topology,
            CommandTopology::Direct(CommandSource::Teleop)
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
