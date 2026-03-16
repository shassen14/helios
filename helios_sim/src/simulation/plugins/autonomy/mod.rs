// helios_sim/src/plugins/autonomy/mod.rs
//
// EstimationPlugin   — spawning + sensor routing + EKF/UKF + odom frames + telemetry.
// MappingPlugin      — architectural seam (run_mapping lives in EstimationPlugin's chain).
// AutonomyPlugin     — trivial combiner kept for backward compatibility.

use crate::prelude::*;

mod components;
pub mod systems;

pub use components::{ControlPipelineComponent, EstimatorComponent, MapperComponent, OdomFrameOf};

use systems::{
    publish_autonomy_telemetry, publish_sensor_telemetry, route_sensor_messages, run_estimation,
    run_mapping, spawn_autonomy_pipeline, spawn_odom_frames, update_odom_frames,
};

// =========================================================================
// == EstimationPlugin
// =========================================================================

/// Handles spawning of all agent pipeline components, sensor routing, EKF/UKF,
/// odom frame maintenance, and telemetry.
pub struct EstimationPlugin;

impl Plugin for EstimationPlugin {
    fn build(&self, app: &mut App) {
        app
            // --- SPAWNING ---
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    spawn_autonomy_pipeline.in_set(SceneBuildSet::ProcessBaseAutonomy),
                    spawn_odom_frames.in_set(SceneBuildSet::ProcessDependentAutonomy),
                ),
            )
            // --- RUNTIME: Estimation phase ---
            // Chain: route fills SensorMailbox → estimation runs → odom frames updated.
            .add_systems(
                FixedUpdate,
                (route_sensor_messages, run_estimation, update_odom_frames)
                    .chain()
                    .in_set(SimulationSet::Estimation)
                    .run_if(in_state(AppState::Running)),
            )
            // --- RUNTIME: Validation phase (cold telemetry path) ---
            .add_systems(
                FixedUpdate,
                (publish_autonomy_telemetry, publish_sensor_telemetry)
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}

// =========================================================================
// == MappingPlugin
// =========================================================================

/// Owns the mapping subsystem — `run_mapping` in `SimulationSet::WorldModeling`.
///
/// `WorldModeling` runs before `Estimation` in the schedule, so `run_mapping`
/// reads the mailbox filled by the previous tick's `route_sensor_messages`.
/// This one-tick latency is standard practice in robotics mapping.
pub struct MappingPlugin;

impl Plugin for MappingPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate,
            run_mapping
                .in_set(SimulationSet::WorldModeling)
                .run_if(in_state(AppState::Running)),
        );
    }
}

// =========================================================================
// == AutonomyPlugin (backward-compatibility combiner)
// =========================================================================

/// Combined estimation + mapping plugin. Identical behavior to the original
/// monolithic `AutonomyPlugin`.
pub struct AutonomyPlugin;

impl Plugin for AutonomyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((EstimationPlugin, MappingPlugin));
    }
}
