// helios_sim/src/plugins/autonomy/mod.rs

use crate::prelude::*;

mod components;
mod systems;

pub use components::{ControlPipelineComponent, EstimatorComponent, MapperComponent, OdomFrameOf};

use systems::{
    autonomy_telemetry_system, estimation_system, mapping_system, route_sensor_messages,
    sensor_telemetry_system, spawn_odom_frames, spawn_world_model_modules, update_odom_frames,
};

pub struct AutonomyPlugin;

impl Plugin for AutonomyPlugin {
    fn build(&self, app: &mut App) {
        app
            // --- SPAWNING ---
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    spawn_world_model_modules.in_set(SceneBuildSet::ProcessBaseAutonomy),
                    // ProcessDependentAutonomy runs after ProcessBaseAutonomy (EstimatorComponent
                    // exists) and before Finalize (where build_static_tf_maps reads all names).
                    spawn_odom_frames.in_set(SceneBuildSet::ProcessDependentAutonomy),
                ),
            )
            // --- RUNTIME: Estimation phase ---
            // route_sensor_messages runs first, populating SensorMailbox.
            // estimation_system and mapping_system then run in parallel (different components).
            // update_odom_frames runs last (needs fresh estimated state).
            .add_systems(
                FixedUpdate,
                (
                    route_sensor_messages,
                    (estimation_system, mapping_system),
                    update_odom_frames,
                )
                    .chain()
                    .in_set(SimulationSet::Estimation)
                    .run_if(in_state(AppState::Running)),
            )
            // --- RUNTIME: Validation phase (cold telemetry path) ---
            // TopicBus writes are isolated here — no hot-path contention.
            .add_systems(
                FixedUpdate,
                (autonomy_telemetry_system, sensor_telemetry_system)
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}
