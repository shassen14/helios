//! Registers built-in path follower factories.

use helios_core::path_following::{
    pure_pursuit::PurePursuitPathFollower, steering_pid::SteeringPidPathFollower, PathFollower,
};

use crate::config::PathFollowingConfig;
use crate::pipeline::builders::path_follower::DefaultPathFollowerInputBuilder;
use crate::pipeline::node::PipelineNode;
use crate::pipeline::nodes::path_follower::PathFollowerNode;

use super::{contexts::PathFollowerBuildContext, AutonomyRegistry};

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_path_follower("PurePursuit", build_pure_pursuit);
    registry.register_path_follower("SteeringPid", build_steering_pid);
}

fn build_pure_pursuit(ctx: PathFollowerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let PathFollowingConfig::PurePursuit {
        max_speed_m_s,
        min_speed_m_s,
        lookahead_distance_m,
        lookahead_time_s,
        goal_radius,
        max_lateral_acceleration,
        ..
    } = ctx.config
    else {
        return Err("build_pure_pursuit received wrong config variant".to_string());
    };

    let follower: Box<dyn PathFollower> = Box::new(PurePursuitPathFollower::new(
        lookahead_distance_m,
        lookahead_time_s,
        goal_radius,
        min_speed_m_s,
        max_speed_m_s,
        max_lateral_acceleration,
        ctx.agent_handle,
    ));

    let input_builder = Box::new(DefaultPathFollowerInputBuilder::new());
    Ok(Box::new(PathFollowerNode::new(
        "pure_pursuit",
        follower,
        input_builder,
        ctx.path_channel,
    )))
}

fn build_steering_pid(ctx: PathFollowerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let PathFollowingConfig::SteeringPid {
        cruise_speed,
        kp,
        ki,
        kd,
        goal_radius,
        lookahead_distance_m,
        ..
    } = ctx.config
    else {
        return Err("build_steering_pid received wrong config variant".to_string());
    };

    let follower: Box<dyn PathFollower> = Box::new(SteeringPidPathFollower::new(
        kp,
        ki,
        kd,
        cruise_speed,
        goal_radius,
        lookahead_distance_m,
        ctx.agent_handle,
    ));

    let input_builder = Box::new(DefaultPathFollowerInputBuilder::new());
    Ok(Box::new(PathFollowerNode::new(
        "steering_pid",
        follower,
        input_builder,
        ctx.path_channel,
    )))
}
