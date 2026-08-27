use crate::{
    config::AllocatorConfig, nodes::allocator::node::AllocatorNode,
    registry::contexts::AllocatorBuildContext, AutonomyRegistry, PipelineNode,
};

use helios_core::control::{
    actuators::ActuatorId,
    allocation::wheeled::{
        ackermann::KinematicAckermannAllocator, drive_torque::WheelTorqueAllocator,
        steer_position::SteerPositionAllocator,
    },
};

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_allocator("KinematicAckermann", build_kinematic_ackermann);
    registry.register_allocator("WheelTorque", build_wheel_torque);
    registry.register_allocator("SteerPosition", build_steer_position);
}

fn build_kinematic_ackermann(ctx: AllocatorBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    // One factory per allocator kind, dispatched by the registry on the config's
    // `kind` string, so `ctx.config` is already this variant. The `else` arm can
    // only fire if the registry maps this kind to the wrong factory — a wiring
    // mistake, not bad user config — so it errors loudly and names what arrived.
    let received_kind = ctx.config.get_kind_str().to_string();
    let AllocatorConfig::KinematicAckermann {
        wheelbase,
        wheel_radius,
        drive,
        steer,
    } = ctx.config
    else {
        return Err(format!(
            "KinematicAckermann allocator factory received a `{received_kind}` config",
        ));
    };

    let allocator = KinematicAckermannAllocator::new(
        wheelbase,
        wheel_radius,
        ActuatorId::new(drive),
        ActuatorId::new(steer),
    );

    Ok(Box::new(AllocatorNode::new(
        ctx.instance_name,
        allocator,
        ctx.input_channel,
        ctx.output_channel,
    )))
}

fn build_wheel_torque(ctx: AllocatorBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    // Consumes a `DriveForce` and emits a single wheel-torque setpoint (`τ = F·r`);
    // pairs with a lateral allocator on a steered vehicle. See
    // `build_kinematic_ackermann` for the `else`-arm rationale.
    let received_kind = ctx.config.get_kind_str().to_string();
    let AllocatorConfig::WheelTorque {
        wheel_radius,
        drive,
    } = ctx.config
    else {
        return Err(format!(
            "WheelTorque allocator factory received a `{received_kind}` config",
        ));
    };

    let allocator = WheelTorqueAllocator::new(wheel_radius, ActuatorId::new(drive));

    Ok(Box::new(AllocatorNode::new(
        ctx.instance_name,
        allocator,
        ctx.input_channel,
        ctx.output_channel,
    )))
}

fn build_steer_position(ctx: AllocatorBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let received_kind = ctx.config.get_kind_str().to_string();
    let AllocatorConfig::SteerPosition { steer } = ctx.config else {
        return Err(format!(
            "SteerPosition allocator factory received a `{received_kind}` config",
        ));
    };

    let allocator = SteerPositionAllocator::new(ActuatorId::new(steer));

    Ok(Box::new(AllocatorNode::new(
        ctx.instance_name,
        allocator,
        ctx.input_channel,
        ctx.output_channel,
    )))
}
