//! Registers built-in controller factories.

use super::input::DefaultControlInputBuilder;
use super::node::ControllerNode;

use crate::config::ControllerConfig;
use crate::pipeline::node::PipelineNode;
use crate::registry::{contexts::ControllerBuildContext, AutonomyRegistry};

use helios_core::control::direct_twist::DirectTwistController;
use helios_core::control::BodyTwistRef;

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_controller("DirectTwist", build_direct_twist);
}

fn build_direct_twist(ctx: ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    // `ControllerConfig` has one variant today, so this destructure is irrefutable.
    // Kept as an explicit binding so adding a second variant turns this into a
    // compile error, forcing the new factory to be written rather than silently
    // mis-dispatching.
    let ControllerConfig::DirectTwist { .. } = ctx.config;

    let controller = DirectTwistController::new();
    let input_builder = Box::new(DefaultControlInputBuilder::<BodyTwistRef>::new());

    Ok(Box::new(ControllerNode::new(
        ctx.instance_name,
        controller,
        input_builder,
        ctx.output_channel,
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::InternalChannel;

    use helios_core::control::commands::BodyTwist;
    use helios_core::data::primitives::FrameHandle;

    fn context(instance_name: &str) -> ControllerBuildContext {
        ControllerBuildContext {
            agent_handle: FrameHandle(0),
            instance_name: instance_name.to_string(),
            config: ControllerConfig::DirectTwist {
                state_source: Default::default(),
            },
            output_channel: InternalChannel::of::<BodyTwist>(),
        }
    }

    // The node name is the config-map key, not the kind: two `DirectTwist`
    // controllers under distinct keys must yield distinct node identities.
    #[test]
    fn node_name_is_the_config_key_not_the_kind() {
        let left = build_direct_twist(context("left_wheels")).unwrap();
        let right = build_direct_twist(context("right_wheels")).unwrap();

        assert_eq!(left.name(), "left_wheels");
        assert_eq!(right.name(), "right_wheels");
    }
}
