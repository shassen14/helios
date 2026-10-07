//! Registers the allocator kinds, each of which builds an [`AllocatorNode`]
//! from a `[nodes.<name>]` entry.

use super::config::{SteerPositionConfig, WheelTorqueConfig};
use super::node::AllocatorNode;

use crate::assembly::{AutonomyRegistry, BuildContext, FactoryOutput};
use crate::port::InternalChannel;

use helios_core::control::actuators::{ActuatorCommand, ActuatorId};
use helios_core::control::allocation::wheeled::drive_torque::WheelTorqueAllocator;
use helios_core::control::allocation::wheeled::steer_position::SteerPositionAllocator;
use helios_core::control::commands::{DriveForce, SteerAngle};

/// The `kind` a profile writes to turn a `DriveForce` into a wheel torque.
pub(crate) const WHEEL_TORQUE_KIND: &str = "WheelTorque";

/// The `kind` a profile writes to turn a `SteerAngle` into a steer position.
pub(crate) const STEER_POSITION_KIND: &str = "SteerPosition";

/// Adds the allocator kinds to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node(WHEEL_TORQUE_KIND, build_wheel_torque)
        .expect("WheelTorque is registered once, by the default registry");
    registry
        .register_node(STEER_POSITION_KIND, build_steer_position)
        .expect("SteerPosition is registered once, by the default registry");
}

fn build_wheel_torque(
    config: WheelTorqueConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    let node = AllocatorNode::new(
        ctx.node_name(),
        WheelTorqueAllocator::new(config.wheel_radius, ActuatorId::new(config.drive)),
        InternalChannel::named::<DriveForce>(config.input),
        InternalChannel::named::<ActuatorCommand>(ctx.node_name()),
    );

    Ok(FactoryOutput::new(Box::new(node)))
}

fn build_steer_position(
    config: SteerPositionConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    let node = AllocatorNode::new(
        ctx.node_name(),
        SteerPositionAllocator::new(ActuatorId::new(config.steer)),
        InternalChannel::named::<SteerAngle>(config.input),
        InternalChannel::named::<ActuatorCommand>(ctx.node_name()),
    );

    Ok(FactoryOutput::new(Box::new(node)))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    use helios_core::control::actuators::{ActuatorDrive, SetpointKind};
    use helios_core::prelude::AgentId;

    use std::collections::HashSet;

    fn context<'a>(name: &'a str, sensors: &'a HashSet<String>) -> BuildContext<'a> {
        BuildContext::new(AgentId::new("test_agent"), name, sensors)
    }

    fn wheel_torque(input: &str, drive: &str) -> WheelTorqueConfig {
        WheelTorqueConfig {
            input: input.to_string(),
            wheel_radius: 0.3,
            drive: drive.to_string(),
        }
    }

    fn steer_position(input: &str, steer: &str) -> SteerPositionConfig {
        SteerPositionConfig {
            input: input.to_string(),
            steer: steer.to_string(),
        }
    }

    #[test]
    fn each_kind_reads_its_fold_and_writes_a_partial_named_after_the_node() {
        let sensors = HashSet::new();
        let cases = [
            (
                build_wheel_torque(
                    wheel_torque("drive_cmd", "rear"),
                    &context("drive", &sensors),
                ),
                ChannelKey::from(InternalChannel::named::<DriveForce>("drive_cmd")),
                "drive",
            ),
            (
                build_steer_position(
                    steer_position("steer_cmd", "front"),
                    &context("steer", &sensors),
                ),
                InternalChannel::named::<SteerAngle>("steer_cmd").into(),
                "steer",
            ),
        ];

        for (output, input, name) in cases {
            let node = output.expect("allocator builds").into_node();
            assert_eq!(node.name(), name);
            assert_eq!(
                node.port_descriptor()
                    .required_inputs()
                    .cloned()
                    .collect::<Vec<_>>(),
                vec![input]
            );
            assert_eq!(
                node.port_descriptor().outputs(),
                vec![ChannelKey::from(InternalChannel::named::<ActuatorCommand>(
                    name
                ))]
            );
        }
    }

    #[test]
    fn each_kinds_output_carries_the_actuator_it_drives_and_the_kind_it_writes() {
        let sensors = HashSet::new();
        let cases = [
            (
                build_wheel_torque(wheel_torque("drive_cmd", "rear"), &context("d", &sensors)),
                "d",
                ActuatorDrive::new(ActuatorId::new("rear"), SetpointKind::Torque),
            ),
            (
                build_steer_position(
                    steer_position("steer_cmd", "front"),
                    &context("s", &sensors),
                ),
                "s",
                ActuatorDrive::new(ActuatorId::new("front"), SetpointKind::Position),
            ),
        ];

        for (output, name, drive) in cases {
            let node = output.expect("allocator builds").into_node();
            let partial = InternalChannel::named::<ActuatorCommand>(name).into();
            assert_eq!(node.port_descriptor().drives(&partial), [drive]);
        }
    }
}
