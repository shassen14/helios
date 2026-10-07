//! Registers the controller kinds, each of which builds a [`ControllerNode`]
//! from a `[nodes.<name>]` entry.

use super::config::{
    BicycleSteerConfig, DirectTwistConfig, LongitudinalVelocityConfig, RoadLoadConfig,
};
use super::input::DefaultControlInputBuilder;
use super::node::ControllerNode;

use crate::assembly::{AutonomyRegistry, BuildContext, FactoryOutput};
use crate::port::InternalChannel;

use helios_core::control::controllers::direct_twist::DirectTwistController;
use helios_core::control::controllers::feedback::longitudinal_velocity::LongitudinalVelocityController;
use helios_core::control::controllers::feedforward::bicycle_steer::BicycleSteerFeedforward;
use helios_core::control::controllers::feedforward::road_load::RoadLoadFeedforward;
use helios_core::control::kernels::siso_pid::SisoPid;
use helios_core::control::{BodyTwistRef, ControlInputs, Controller};
use helios_core::spatial::FrameId;

/// The `kind` a profile writes to pass the reference's velocity through as a
/// `BodyTwist`.
pub(crate) const DIRECT_TWIST_KIND: &str = "DirectTwist";

/// The `kind` a profile writes to get a forward-speed PID emitting a
/// `DriveForce`.
pub(crate) const LONGITUDINAL_VELOCITY_KIND: &str = "LongitudinalVelocity";

/// The `kind` a profile writes to get the road-load `DriveForce` feedforward.
pub(crate) const ROAD_LOAD_KIND: &str = "RoadLoad";

/// The `kind` a profile writes to get the bicycle-model `SteerAngle`
/// feedforward.
pub(crate) const BICYCLE_STEER_KIND: &str = "BicycleSteer";

/// Adds the controller kinds to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node(DIRECT_TWIST_KIND, build_direct_twist)
        .expect("DirectTwist is registered once, by the default registry");
    registry
        .register_node(LONGITUDINAL_VELOCITY_KIND, build_longitudinal_velocity)
        .expect("LongitudinalVelocity is registered once, by the default registry");
    registry
        .register_node(ROAD_LOAD_KIND, build_road_load)
        .expect("RoadLoad is registered once, by the default registry");
    registry
        .register_node(BICYCLE_STEER_KIND, build_bicycle_steer)
        .expect("BicycleSteer is registered once, by the default registry");
}

fn build_direct_twist(
    _config: DirectTwistConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    Ok(controller_output(DirectTwistController::new(), ctx))
}

fn build_longitudinal_velocity(
    config: LongitudinalVelocityConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    // Feedback leg of the longitudinal loop. The controlled frame is this
    // agent's `base_link`.
    let pid = SisoPid::new(
        config.proportional_gain,
        config.integral_gain,
        config.derivative_gain,
    )
    .with_integral_clamp(config.integral_clamp);
    let controller =
        LongitudinalVelocityController::new(pid, FrameId::base_link(ctx.agent().clone()));

    Ok(controller_output(controller, ctx))
}

fn build_road_load(config: RoadLoadConfig, ctx: &BuildContext) -> Result<FactoryOutput, String> {
    // Feedforward leg of the longitudinal loop: the law reads only the
    // reference. `DefaultControlInputBuilder` still lists the state estimate as
    // a required input, so the node waits for one even though it never reads
    // it. Harmless while an estimator always runs; a reference-only input
    // builder is the honest fix, deferred.
    Ok(controller_output(
        RoadLoadFeedforward::new(config.c_roll, config.c_drag),
        ctx,
    ))
}

fn build_bicycle_steer(
    config: BicycleSteerConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    Ok(controller_output(
        BicycleSteerFeedforward::new(config.wheelbase),
        ctx,
    ))
}

/// Wraps `controller` in a node that tracks the guidance reference and
/// publishes its command on a channel named after the node, typed by the
/// controller's own output. Which fold, if any, the command joins is the
/// stack's business, so the node is the same either way.
fn controller_output<C>(controller: C, ctx: &BuildContext) -> FactoryOutput
where
    C: Controller<Inputs = ControlInputs<BodyTwistRef>> + 'static,
    C::Out: Send + Sync + 'static,
{
    let node = ControllerNode::new(
        ctx.node_name(),
        controller,
        Box::new(DefaultControlInputBuilder::<BodyTwistRef>::new()),
        InternalChannel::named::<C::Out>(ctx.node_name()),
    );

    FactoryOutput::new(Box::new(node))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    use helios_core::control::commands::{BodyTwist, DriveForce, SteerAngle};
    use helios_core::prelude::AgentId;

    use std::collections::HashSet;

    fn context<'a>(name: &'a str, sensors: &'a HashSet<String>) -> BuildContext<'a> {
        BuildContext::new(AgentId::new("test_agent"), name, sensors)
    }

    fn output_of(output: FactoryOutput) -> Vec<ChannelKey> {
        output.into_node().port_descriptor().outputs().to_vec()
    }

    #[test]
    fn node_name_is_the_table_key_not_the_kind() {
        let sensors = HashSet::new();
        let left = build_direct_twist(DirectTwistConfig {}, &context("left_wheels", &sensors))
            .expect("DirectTwist builds")
            .into_node();
        let right = build_direct_twist(DirectTwistConfig {}, &context("right_wheels", &sensors))
            .expect("DirectTwist builds")
            .into_node();

        assert_eq!(left.name(), "left_wheels");
        assert_eq!(right.name(), "right_wheels");
    }

    #[test]
    fn each_kind_writes_its_own_command_type_named_after_the_node() {
        let sensors = HashSet::new();
        let speed = LongitudinalVelocityConfig {
            proportional_gain: 1.0,
            integral_gain: 0.0,
            derivative_gain: 0.0,
            integral_clamp: 0.0,
        };
        let road_load = RoadLoadConfig {
            c_roll: 1.0,
            c_drag: 0.0,
        };

        let cases = [
            (
                output_of(
                    build_direct_twist(DirectTwistConfig {}, &context("twist", &sensors)).unwrap(),
                ),
                ChannelKey::from(InternalChannel::named::<BodyTwist>("twist")),
            ),
            (
                output_of(build_longitudinal_velocity(speed, &context("speed", &sensors)).unwrap()),
                InternalChannel::named::<DriveForce>("speed").into(),
            ),
            (
                output_of(build_road_load(road_load, &context("ff", &sensors)).unwrap()),
                InternalChannel::named::<DriveForce>("ff").into(),
            ),
            (
                output_of(
                    build_bicycle_steer(
                        BicycleSteerConfig { wheelbase: 2.5 },
                        &context("steer", &sensors),
                    )
                    .unwrap(),
                ),
                InternalChannel::named::<SteerAngle>("steer").into(),
            ),
        ];

        for (outputs, expected) in cases {
            assert_eq!(outputs, vec![expected]);
        }
    }

    #[test]
    fn a_controller_declares_no_outside_input() {
        // Its reference and state both come from inside the graph.
        let sensors = HashSet::new();
        let output = build_bicycle_steer(
            BicycleSteerConfig { wheelbase: 2.5 },
            &context("steer", &sensors),
        )
        .expect("BicycleSteer builds");

        assert!(output.outside_inputs().is_empty());
    }
}
