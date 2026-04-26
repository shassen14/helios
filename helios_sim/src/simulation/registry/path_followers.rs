// helios_sim/src/simulation/registry/path_followers.rs
//
// Registers all built-in path follower factories into AutonomyRegistry.
// Add new algorithms here — zero spawning systems change.

use bevy::prelude::*;
use helios_core::path_following::pure_pursuit::PurePursuitPathFollower;
use helios_core::path_following::steering_pid::SteeringPidPathFollower;
use helios_core::types::FrameHandle;

use super::{AutonomyRegistry, PathFollowerBuildContext};
use crate::simulation::config::structs::PathFollowingConfig;

pub struct DefaultPathFollowersPlugin;

impl Plugin for DefaultPathFollowersPlugin {
    fn build(&self, app: &mut App) {
        let Some(mut registry) = app.world_mut().get_resource_mut::<AutonomyRegistry>() else {
            error!("DefaultPathFollowersPlugin: AutonomyRegistry resource not found. Add AutonomyRegistryPlugin first.");
            return;
        };

        // --- SteeringPid ---
        registry.register_path_follower("SteeringPid", |ctx: PathFollowerBuildContext| {
            if let PathFollowingConfig::SteeringPid {
                cruise_speed,
                kp,
                ki,
                kd,
                goal_radius,
                lookahead_distance_m,
                ..
            } = ctx.path_following_cfg
            {
                let agent_handle = FrameHandle::from_entity(ctx.agent_entity);
                Ok(Box::new(SteeringPidPathFollower::new(
                    kp,
                    ki,
                    kd,
                    cruise_speed,
                    goal_radius,
                    lookahead_distance_m,
                    agent_handle,
                )))
            } else {
                Err("SteeringPid path follower factory received wrong config variant".into())
            }
        });

        // --- PurePursuit ---
        registry.register_path_follower("PurePursuit", |ctx: PathFollowerBuildContext| {
            if let PathFollowingConfig::PurePursuit {
                max_speed_m_s,
                min_speed_m_s,
                lookahead_distance_m,
                goal_radius,
                max_lateral_acceleration,
                ..
            } = ctx.path_following_cfg
            {
                let agent_handle = FrameHandle::from_entity(ctx.agent_entity);
                Ok(Box::new(PurePursuitPathFollower::new(
                    0.0, // wheelbase: unused in current compute() implementation
                    lookahead_distance_m,
                    None, // lookahead_time: stub not yet implemented
                    goal_radius,
                    min_speed_m_s,
                    max_speed_m_s,
                    max_lateral_acceleration,
                    agent_handle,
                )))
            } else {
                Err("PurePursuit factory received wrong PathFollowingConfig variant".into())
            }
        });
    }
}
