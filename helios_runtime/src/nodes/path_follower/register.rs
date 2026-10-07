//! Registers the `PurePursuit` and `SteeringPid` kinds, which build a
//! [`PathFollowerNode`] from a `[nodes.<name>]` entry.

use super::config::{PurePursuitConfig, SteeringPidConfig};
use super::input::DefaultPathFollowerInputBuilder;
use super::node::PathFollowerNode;

use crate::assembly::{AutonomyRegistry, BuildContext, FactoryOutput};
use crate::port::InternalChannel;

use helios_core::control::BodyTwistRef;
use helios_core::following::{
    pure_pursuit::PurePursuitPathFollower, steering_pid::SteeringPidPathFollower, PathFollower,
};
use helios_core::interchange::path::Path;

/// The `kind` a profile writes to get a pure-pursuit follower node.
pub(crate) const PURE_PURSUIT_KIND: &str = "PurePursuit";

/// The `kind` a profile writes to get a steering-PID follower node.
pub(crate) const STEERING_PID_KIND: &str = "SteeringPid";

/// Adds the path-follower kinds to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node(PURE_PURSUIT_KIND, build_pure_pursuit)
        .expect("PurePursuit is registered once, by the default registry");
    registry
        .register_node(STEERING_PID_KIND, build_steering_pid)
        .expect("SteeringPid is registered once, by the default registry");
}

fn build_pure_pursuit(
    config: PurePursuitConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    let follower: Box<dyn PathFollower<Reference = BodyTwistRef>> =
        Box::new(PurePursuitPathFollower::new(
            config.lookahead_distance_m,
            config.lookahead_time_s,
            config.goal_radius,
            config.min_speed_m_s,
            config.max_speed_m_s,
            config.max_lateral_acceleration,
            ctx.agent().clone(),
        ));

    Ok(follower_output(follower, &config.path, ctx))
}

fn build_steering_pid(
    config: SteeringPidConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    let follower: Box<dyn PathFollower<Reference = BodyTwistRef>> =
        Box::new(SteeringPidPathFollower::new(
            config.kp,
            config.ki,
            config.kd,
            config.cruise_speed,
            config.goal_radius,
            config.lookahead_distance_m,
            ctx.agent().clone(),
        ));

    Ok(follower_output(follower, &config.path, ctx))
}

/// Wraps `follower` in a node that reads `Path` from `path` and publishes its
/// reference on a channel named after the node. Which seam, if any, the
/// reference feeds is the stack's business, so the node is the same either way.
fn follower_output(
    follower: Box<dyn PathFollower<Reference = BodyTwistRef>>,
    path: &str,
    ctx: &BuildContext,
) -> FactoryOutput {
    let node = PathFollowerNode::new(
        ctx.node_name(),
        follower,
        Box::new(DefaultPathFollowerInputBuilder::new()),
        InternalChannel::named::<Path>(path),
        InternalChannel::named::<BodyTwistRef>(ctx.node_name()),
    );

    FactoryOutput::new(Box::new(node))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    use helios_core::prelude::AgentId;

    use std::collections::HashSet;

    fn pure_pursuit(path: &str) -> PurePursuitConfig {
        toml::from_str(&format!(
            "path = \"{path}\"\nmax_speed_m_s = 5.0\nmin_speed_m_s = 0.5"
        ))
        .expect("a valid pure-pursuit section")
    }

    fn steering_pid(path: &str) -> SteeringPidConfig {
        toml::from_str(&format!("path = \"{path}\"\ncruise_speed = 2.0"))
            .expect("a valid steering-PID section")
    }

    fn context<'a>(name: &'a str, sensors: &'a HashSet<String>) -> BuildContext<'a> {
        BuildContext::new(AgentId::new("test_agent"), name, sensors)
    }

    #[test]
    fn node_name_is_the_table_key_not_the_kind() {
        let sensors = HashSet::new();
        let fast = build_pure_pursuit(pure_pursuit("local_path"), &context("fast", &sensors))
            .expect("pure pursuit builds")
            .into_node();
        let careful = build_steering_pid(steering_pid("local_path"), &context("careful", &sensors))
            .expect("steering PID builds")
            .into_node();

        assert_eq!(fast.name(), "fast");
        assert_eq!(careful.name(), "careful");
    }

    #[test]
    fn reads_the_named_path_and_writes_a_reference_named_after_the_node() {
        let sensors = HashSet::new();
        let node = build_pure_pursuit(pure_pursuit("global_path"), &context("pp", &sensors))
            .expect("pure pursuit builds")
            .into_node();
        let descriptor = node.port_descriptor();

        let path: ChannelKey = InternalChannel::named::<Path>("global_path").into();
        let reference: ChannelKey = InternalChannel::named::<BodyTwistRef>("pp").into();
        assert!(descriptor.required_inputs().any(|key| *key == path));
        assert_eq!(descriptor.outputs(), std::slice::from_ref(&reference));
    }

    #[test]
    fn a_follower_declares_no_outside_input() {
        // Its path comes from a planner and its state from the estimator, both
        // inside the graph.
        let sensors = HashSet::new();
        let output = build_steering_pid(steering_pid("local_path"), &context("pid", &sensors))
            .expect("steering PID builds");

        assert!(output.outside_inputs().is_empty());
    }
}
