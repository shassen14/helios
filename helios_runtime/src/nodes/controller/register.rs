//! Registers built-in controller factories.

use super::input::DefaultControlInputBuilder;
use super::node::ControllerNode;

use crate::config::ControllerConfig;
use crate::pipeline::node::PipelineNode;
use crate::registry::{contexts::ControllerBuildContext, AutonomyRegistry};

use helios_core::control::controllers::direct_twist::DirectTwistController;
use helios_core::control::controllers::feedback::longitudinal_velocity::LongitudinalVelocityController;
use helios_core::control::controllers::feedforward::road_load::RoadLoadFeedforward;
use helios_core::control::kernels::siso_pid::SisoPid;
use helios_core::control::BodyTwistRef;
use helios_core::frames::FrameId;

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_controller("DirectTwist", build_direct_twist);
    registry.register_controller("LongitudinalVelocity", build_longitudinal_velocity);
    registry.register_controller("RoadLoad", build_road_load);
}

fn build_direct_twist(ctx: ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    // One factory per controller kind, dispatched by the registry on the config's
    // `kind` string, so `ctx.config` is already this variant. The `else` arm can
    // only fire if the registry maps this kind to the wrong factory — a wiring
    // mistake, not bad user config — so it errors loudly and names what arrived.
    let received_kind = ctx.config.get_kind_str().to_string();
    let ControllerConfig::DirectTwist { .. } = ctx.config else {
        return Err(format!(
            "DirectTwist controller factory received a `{received_kind}` config",
        ));
    };

    let controller = DirectTwistController::new();
    let input_builder = Box::new(DefaultControlInputBuilder::<BodyTwistRef>::new());

    Ok(Box::new(ControllerNode::new(
        ctx.instance_name,
        controller,
        input_builder,
        ctx.output_channel,
    )))
}

fn build_longitudinal_velocity(
    ctx: ControllerBuildContext,
) -> Result<Box<dyn PipelineNode>, String> {
    // Feedback leg of the longitudinal loop: a SISO PID on body-forward speed
    // emitting a `DriveForce`. The controlled frame is this agent's body frame,
    // derived from the handle the host resolved into the build context.
    let received_kind = ctx.config.get_kind_str().to_string();
    let ControllerConfig::LongitudinalVelocity {
        proportional_gain,
        integral_gain,
        derivative_gain,
        ..
    } = ctx.config
    else {
        return Err(format!(
            "LongitudinalVelocity controller factory received a `{received_kind}` config",
        ));
    };

    let pid = SisoPid::new(proportional_gain, integral_gain, derivative_gain);
    let controller = LongitudinalVelocityController::new(pid, FrameId::Body(ctx.agent_handle));
    let input_builder = Box::new(DefaultControlInputBuilder::<BodyTwistRef>::new());

    Ok(Box::new(ControllerNode::new(
        ctx.instance_name,
        controller,
        input_builder,
        ctx.output_channel,
    )))
}

fn build_road_load(ctx: ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    // Feedforward leg of the longitudinal loop: the open-loop road-load force
    // (rolling + drag) for the reference speed, summed with the feedback leg
    // downstream. Pure feedforward — the controller reads only the reference.
    //
    // Caveat from the wrapper, not the controller: `DefaultControlInputBuilder`
    // lists `FrameAwareState` as a *required* channel, so the node won't fire
    // until a state estimate is on the bus even though this law never reads one.
    // Harmless while an estimator always runs; a reference-only input builder is
    // the honest fix, deferred.
    let received_kind = ctx.config.get_kind_str().to_string();
    let ControllerConfig::RoadLoad { c_roll, c_drag } = ctx.config else {
        return Err(format!(
            "RoadLoad controller factory received a `{received_kind}` config",
        ));
    };

    let controller = RoadLoadFeedforward::new(c_roll, c_drag);
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
