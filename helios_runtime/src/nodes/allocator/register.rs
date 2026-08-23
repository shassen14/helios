use crate::{
    config::AllocatorConfig, nodes::allocator::node::AllocatorNode,
    registry::contexts::AllocatorBuildContext, AutonomyRegistry, PipelineNode,
};

use helios_core::control::{
    actuators::ActuatorId, allocation::wheeled::ackermann::KinematicAckermannAllocator,
};

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_allocator("KinematicAckermann", build_kinematic_ackermann);
}

fn build_kinematic_ackermann(ctx: AllocatorBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let AllocatorConfig::KinematicAckermann {
        wheelbase,
        wheel_radius,
        drive,
        steer,
    } = ctx.config;

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
