// helios_sim/src/simulation/registry/slam.rs
//
// Registers all known SLAM systems with the AutonomyRegistry.
// SLAM systems implement SlamSystem (= StateEstimator + Mapper), enabling the
// CombinedSlam WorldModelConfig variant to run a single unified algorithm.
//
// To add a SLAM system (e.g., LIO-SAM):
//   1. Implement SlamSystem in helios_core.
//   2. Add register_slam("LioSam", build_lio_sam) below.
//   Zero spawning systems change.

use bevy::prelude::*;
use helios_core::slam::SlamSystem;

use super::{AutonomyRegistry, SlamBuildContext};

pub struct DefaultSlamPlugin;

impl Plugin for DefaultSlamPlugin {
    fn build(&self, app: &mut App) {
        app.world_mut()
            .resource_mut::<AutonomyRegistry>()
            .register_slam("EkfSlam", build_ekf_slam)
            .register_slam("FactorGraphSlam", build_factor_graph_slam);
    }
}

fn build_ekf_slam(_ctx: SlamBuildContext) -> Option<Box<dyn SlamSystem>> {
    warn!("AutonomyRegistry: EKF-SLAM not yet implemented.");
    None
}

fn build_factor_graph_slam(_ctx: SlamBuildContext) -> Option<Box<dyn SlamSystem>> {
    warn!("AutonomyRegistry: Factor Graph SLAM not yet implemented.");
    None
}
