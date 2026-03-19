use bevy::prelude::*;

mod cache;
mod components;
pub mod gizmos;
pub mod key_action_registry;
pub mod keybindings;
pub mod ui;

pub use components::{
    DebugLegendNode, DebugSensorCache, DebugVisualizationConfig, PathTrail, TfLabelEntities,
};
pub use key_action_registry::{DebugToggle, KeyAction, KeyActionRegistry};

use crate::prelude::AppState;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::profile::CapabilitySet;
use key_action_registry::{parse_key_code, KeyAction as KA};

/// Default key assignments that are ALWAYS registered (profile-independent).
const ALWAYS_ACTIONS: &[(&str, KeyCode, &str, DebugToggle)] = &[
    (
        "toggle_legend",
        KeyCode::KeyH,
        "H  Legend",
        DebugToggle::Legend,
    ),
    (
        "toggle_pose",
        KeyCode::F1,
        "F1 Pose Gimbals",
        DebugToggle::Pose,
    ),
    (
        "toggle_velocity",
        KeyCode::F4,
        "F4 Velocity",
        DebugToggle::Velocity,
    ),
    (
        "toggle_path_trail",
        KeyCode::F6,
        "F6 Path Trail",
        DebugToggle::PathTrail,
    ),
    (
        "toggle_tf_frames",
        KeyCode::F8,
        "F8 TF Frames",
        DebugToggle::TfFrames,
    ),
];

const SENSOR_ACTIONS: &[(&str, KeyCode, &str, DebugToggle)] = &[(
    "toggle_point_cloud",
    KeyCode::F3,
    "F3 Point Cloud",
    DebugToggle::PointCloud,
)];

const ESTIMATION_ACTIONS: &[(&str, KeyCode, &str, DebugToggle)] = &[
    (
        "toggle_covariance",
        KeyCode::F2,
        "F2 Covariance",
        DebugToggle::Covariance,
    ),
    (
        "toggle_error_line",
        KeyCode::F5,
        "F5 Est. Error",
        DebugToggle::ErrorLine,
    ),
];

const MAPPING_ACTIONS: &[(&str, KeyCode, &str, DebugToggle)] = &[(
    "toggle_occupancy_grid",
    KeyCode::F7,
    "F7 Occupancy Grid",
    DebugToggle::OccupancyGrid,
)];

const PLANNING_ACTIONS: &[(&str, KeyCode, &str, DebugToggle)] = &[(
    "toggle_planned_path",
    KeyCode::F9,
    "F9 Planned Path",
    DebugToggle::PlannedPath,
)];

/// HUD toggle (C) and state-source toggle (T).
/// C is registered for any profile with control or estimation.
/// T is registered only when both control and estimation are active.
const CONTROL_ACTIONS: &[(&str, KeyCode, &str, DebugToggle)] = &[
    (
        "toggle_vehicle_hud",
        KeyCode::KeyC,
        "C  Vehicle HUD",
        DebugToggle::VehicleHud,
    ),
    (
        "toggle_state_source",
        KeyCode::KeyT,
        "T  State Source",
        DebugToggle::StateSource,
    ),
];

/// Build the registry from action tables, gated by `CapabilitySet`.
/// Runs once on entering Running state (before other startup systems read the registry).
fn build_key_registry(
    mut registry: ResMut<KeyActionRegistry>,
    scenario: Res<ScenarioConfig>,
    capabilities: Res<CapabilitySet>,
) {
    let overrides = &scenario.debug.keybindings.0;
    register_actions(&mut registry, overrides, ALWAYS_ACTIONS);

    if capabilities.sensors() {
        register_actions(&mut registry, overrides, SENSOR_ACTIONS);
    }
    if capabilities.estimation() {
        register_actions(&mut registry, overrides, ESTIMATION_ACTIONS);
    }
    if capabilities.mapping() {
        register_actions(&mut registry, overrides, MAPPING_ACTIONS);
    }
    if capabilities.planning() {
        register_actions(&mut registry, overrides, PLANNING_ACTIONS);
    }
    if capabilities.control() || capabilities.estimation() {
        register_actions(&mut registry, overrides, &CONTROL_ACTIONS[..1]); // C key only
    }
    if capabilities.control() && capabilities.estimation() {
        register_actions(&mut registry, overrides, &CONTROL_ACTIONS[1..]); // T key only
    }
}

fn apply_debug_config(scenario: Res<ScenarioConfig>, mut viz: ResMut<DebugVisualizationConfig>) {
    let d = &scenario.debug;
    viz.show_pose_gimbals = d.show_pose_gimbals;
    viz.show_covariance = d.show_covariance;
    viz.show_point_cloud = d.show_point_cloud;
    viz.show_velocity = d.show_velocity;
    viz.show_error_line = d.show_error_line;
    viz.show_path_trail = d.show_path_trail;
    viz.show_occupancy_grid = d.show_occupancy_grid;
    viz.show_tf_frames = d.show_tf_frames;
    viz.show_planned_path = d.show_planned_path;
    viz.show_legend = d.show_legend;
    viz.show_vehicle_hud = d.show_vehicle_hud;
}

/// Top-level plugin that provides all debug visualization tooling.
pub struct DebuggingPlugin;

impl Plugin for DebuggingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugVisualizationConfig>()
            .init_resource::<DebugSensorCache>()
            .init_resource::<TfLabelEntities>()
            .init_resource::<KeyActionRegistry>()
            .add_systems(
                OnEnter(AppState::Running),
                (
                    build_key_registry,
                    apply_debug_config,
                    ui::legend::spawn_debug_legend,
                    ui::vehicle_hud::spawn_vehicle_hud,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                (
                    keybindings::handle_debug_keybindings,
                    cache::cache_sensor_data,
                    ui::legend::update_legend_text,
                    ui::vehicle_hud::update_vehicle_hud,
                    ui::vehicle_hud::toggle_state_source,
                    gizmos::pose::draw_ground_truth_gimbals,
                    gizmos::pose::draw_estimated_pose_gimbals,
                    gizmos::covariance::draw_covariance_ellipsoid,
                    gizmos::point_cloud::draw_point_cloud,
                    gizmos::velocity::draw_velocity_vector,
                    gizmos::error::draw_estimation_error_line,
                    gizmos::trail::draw_path_trail,
                    gizmos::occupancy::draw_occupancy_grid,
                    gizmos::tf_frames::draw_tf_frames,
                    gizmos::tf_frames::update_tf_labels,
                    gizmos::planned_path::draw_planned_path,
                )
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(
                FixedUpdate,
                gizmos::trail::update_path_trail
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}

// ---------------------------------------------------------------------------
// Internal key registration helper.
// ---------------------------------------------------------------------------

fn register_actions(
    registry: &mut KeyActionRegistry,
    overrides: &std::collections::HashMap<String, String>,
    actions: &[(&'static str, KeyCode, &'static str, DebugToggle)],
) {
    for &(id, default_key, label, toggle) in actions {
        let bound_key = overrides
            .get(id)
            .and_then(|s| parse_key_code(s))
            .unwrap_or(default_key);
        if let Some(existing) = registry.0.iter().find(|a| a.bound_key == bound_key) {
            warn!(
                "Key conflict: {:?} already bound to '{}', skipping '{}'",
                bound_key, existing.id, id
            );
            continue;
        }
        registry.0.push(KA {
            id,
            bound_key,
            label,
            toggle,
        });
    }
}
