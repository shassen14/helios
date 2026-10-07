//! Registers the `TwistTeleop` kind, which builds a [`TwistTeleopNode`] from a
//! `[nodes.<name>]` entry.

use super::config::TwistTeleopConfig;
use super::node::{TwistScale, TwistTeleopNode};

use crate::assembly::{AutonomyRegistry, BuildContext, FactoryOutput};
use crate::channels::control;
use crate::port::InternalChannel;

use helios_core::control::commands::TwistIntent;
use helios_core::control::BodyTwistRef;

/// The `kind` a profile writes to get a teleop mapper for a velocity body.
pub(crate) const TWIST_TELEOP_KIND: &str = "TwistTeleop";

/// Adds the `TwistTeleop` kind to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node(TWIST_TELEOP_KIND, build_twist_teleop)
        .expect("TwistTeleop is registered once, by the default registry");
}

/// Builds the mapper for one entry. It publishes its reference on a channel
/// named after the node, and declares the operator's intent as an outside
/// input: the host writes it, and nothing in the graph does.
fn build_twist_teleop(
    config: TwistTeleopConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    let intent = control::intent::<TwistIntent>();
    let node = TwistTeleopNode::new(
        ctx.node_name(),
        intent.clone(),
        InternalChannel::named::<BodyTwistRef>(ctx.node_name()),
        TwistScale {
            surge: config.surge,
            sway: config.sway,
            heave: config.heave,
            roll: config.roll,
            pitch: config.pitch,
            yaw: config.yaw,
        },
    );

    Ok(FactoryOutput::new(Box::new(node)).with_outside_input(intent))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    use helios_core::prelude::AgentId;

    use std::collections::HashSet;

    fn build(name: &str) -> FactoryOutput {
        let config: TwistTeleopConfig =
            toml::from_str("surge = 5.0\nyaw = 1.0").expect("a car's section parses");
        let sensors = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("test_agent"), name, &sensors);
        build_twist_teleop(config, &ctx).expect("the mapper builds")
    }

    #[test]
    fn reads_the_intent_and_writes_a_reference_named_after_the_node() {
        let node = build("operator").into_node();
        let descriptor = node.port_descriptor();

        let intent: ChannelKey = control::intent::<TwistIntent>().into();
        let reference: ChannelKey = InternalChannel::named::<BodyTwistRef>("operator").into();
        assert_eq!(node.name(), "operator");
        assert_eq!(
            descriptor.required_inputs().cloned().collect::<Vec<_>>(),
            vec![intent]
        );
        assert_eq!(descriptor.outputs(), std::slice::from_ref(&reference));
    }

    #[test]
    fn intent_is_declared_as_the_outside_input_the_node_reads() {
        let output = build("operator");

        let intent: ChannelKey = control::intent::<TwistIntent>().into();
        assert_eq!(output.outside_inputs(), std::slice::from_ref(&intent));
    }
}
