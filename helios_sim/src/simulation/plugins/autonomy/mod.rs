// helios_sim/src/plugins/autonomy/mod.rs
//
// EstimationPlugin   — spawning + sensor routing + EKF/UKF + odom frames + telemetry.
// MappingPlugin      — architectural seam (mapping_system lives in EstimationPlugin's chain).
// AutonomyPlugin     — trivial combiner kept for backward compatibility.

use crate::prelude::*;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::plugins::debugging::key_action_registry::KeyActionRegistry;
use crate::simulation::plugins::debugging::{register_actions, DebugToggle};

mod components;
pub mod systems;

pub use components::{ControlPipelineComponent, EstimatorComponent, MapperComponent, OdomFrameOf};

use systems::{
    autonomy_telemetry_system, estimation_system, mapping_system, route_sensor_messages,
    sensor_telemetry_system, spawn_odom_frames, spawn_world_model_modules, update_odom_frames,
};

fn register_estimation_keys(
    mut registry: ResMut<KeyActionRegistry>,
    scenario: Res<ScenarioConfig>,
) {
    let overrides = &scenario.debug.keybindings.0;
    register_actions(
        &mut registry,
        overrides,
        &[
            ("toggle_covariance", KeyCode::F2, "F2 Covariance",   DebugToggle::Covariance),
            ("toggle_error_line", KeyCode::F5, "F5 Est. Error",   DebugToggle::ErrorLine),
        ],
    );
}

fn register_mapping_keys(
    mut registry: ResMut<KeyActionRegistry>,
    scenario: Res<ScenarioConfig>,
) {
    let overrides = &scenario.debug.keybindings.0;
    register_actions(
        &mut registry,
        overrides,
        &[
            ("toggle_occupancy_grid", KeyCode::F7, "F7 Occupancy Grid", DebugToggle::OccupancyGrid),
        ],
    );
}

// =========================================================================
// == EstimationPlugin
// =========================================================================

/// Handles spawning of all agent pipeline components, sensor routing, EKF/UKF,
/// odom frame maintenance, and telemetry.
///
/// Owns `mapping_system` inside its chain to guarantee route→estimate→map→odom
/// write/read ordering. `MappingPlugin` is an architectural seam only.
pub struct EstimationPlugin;

impl Plugin for EstimationPlugin {
    fn build(&self, app: &mut App) {
        app
            // Register estimation-specific debug keys when the simulation starts.
            .add_systems(OnEnter(AppState::Running), register_estimation_keys)
            // --- SPAWNING ---
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    spawn_world_model_modules.in_set(SceneBuildSet::ProcessBaseAutonomy),
                    spawn_odom_frames.in_set(SceneBuildSet::ProcessDependentAutonomy),
                ),
            )
            // --- RUNTIME: Estimation phase ---
            // Chain: route fills SensorMailbox → estimation and mapping run in parallel
            // (different ECS components) → odom frames updated from fresh state.
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
            .add_systems(
                FixedUpdate,
                (autonomy_telemetry_system, sensor_telemetry_system)
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}

// =========================================================================
// == MappingPlugin
// =========================================================================

/// Architectural seam for the mapping subsystem.
///
/// Currently empty — `EstimationPlugin` owns `mapping_system` inside its chain
/// for correct write/read ordering with `route_sensor_messages`. This plugin
/// signals to `ProfiledSimulationPlugin` that mapping is requested; a future
/// refactor will move the mapping systems here once proper intra-set ordering
/// is established.
pub struct MappingPlugin;

impl Plugin for MappingPlugin {
    fn build(&self, app: &mut App) {
        // Register mapping-specific debug key.
        app.add_systems(OnEnter(AppState::Running), register_mapping_keys);
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
