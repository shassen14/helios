// helios_sim/src/simulation/plugins/planning/interaction.rs
//
// Agent selection (left-click) and goal setting (right-click) interaction systems.

use std::collections::HashMap;

use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use helios_core::planning::types::PlannerGoal;
use nalgebra::Vector2;

use crate::simulation::core::events::GoalCommandEvent;
use crate::simulation::core::transforms::EnuVector;
use crate::simulation::plugins::autonomy::ControlPipelineComponent;

// =========================================================================
// == Resources ==
// =========================================================================

/// The currently selected agent entity. Set by left-click; cleared by Escape.
#[derive(Resource, Default)]
pub struct SelectedAgent(pub Option<Entity>);

/// Most recently commanded goal position per agent (for gizmo rendering).
#[derive(Resource, Default)]
pub struct GoalRegistry(pub HashMap<Entity, PlannerGoal>);

// =========================================================================
// == Systems ==
// =========================================================================

const PICK_RADIUS_M: f32 = 6.0;

/// Left-click near an agent to select it. Escape clears the selection.
pub fn agent_select_system(
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    agents: Query<(Entity, &GlobalTransform, &Name), With<ControlPipelineComponent>>,
    mut selected: ResMut<SelectedAgent>,
) {
    if keyboard.just_pressed(KeyCode::Escape) {
        if selected.0.is_some() {
            info!("[Interaction] Agent deselected");
            selected.0 = None;
        }
        return;
    }

    if !mouse_buttons.just_pressed(MouseButton::Left) {
        return;
    }

    let Ok(window) = windows.single() else { return };
    let Ok((camera, cam_gt)) = camera_query.single() else {
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(cam_gt, cursor) else {
        return;
    };

    // Find agent closest to the click ray within PICK_RADIUS_M.
    let mut best: Option<(Entity, f32, String)> = None;

    for (entity, gt, name) in &agents {
        let pos = gt.translation();
        let v = pos - ray.origin;
        let t = v.dot(*ray.direction).max(0.0);
        let on_ray = ray.origin + *ray.direction * t;
        let dist = (pos - on_ray).length();

        if dist <= PICK_RADIUS_M {
            if best.as_ref().map_or(true, |(_, d, _)| dist < *d) {
                best = Some((entity, dist, name.to_string()));
            }
        }
    }

    match best {
        Some((entity, _, name)) => {
            selected.0 = Some(entity);
            info!(
                "[Interaction] Selected '{}'  — right-click to set goal, Esc to deselect",
                name
            );
        }
        None => {
            if selected.0.is_some() {
                selected.0 = None;
                info!("[Interaction] Deselected (no agent at click)");
            }
        }
    }
}

/// Right-click (with an agent selected) to place a navigation goal on the ground plane.
pub fn click_goal_system(
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    selected: Res<SelectedAgent>,
    mut goal_events: EventWriter<GoalCommandEvent>,
    mut goal_registry: ResMut<GoalRegistry>,
) {
    let Some(agent_entity) = selected.0 else {
        return;
    };

    if !mouse_buttons.just_pressed(MouseButton::Right) {
        return;
    }

    let Ok(window) = windows.single() else { return };
    let Ok((camera, cam_gt)) = camera_query.single() else {
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(cam_gt, cursor) else {
        return;
    };

    // Intersect click ray with the ground plane (y = 0 in Bevy world space).
    let dir_y = ray.direction.y;
    if dir_y.abs() < 1e-6 {
        return; // Ray is parallel to the ground.
    }
    let t = -ray.origin.y / dir_y;
    if t < 0.0 {
        return; // Ground is behind the camera.
    }

    let hit = ray.origin + *ray.direction * t;

    // Convert Bevy ground-plane hit to ENU 2D position.
    let hit_enu = EnuVector::from(hit).0;
    let goal = PlannerGoal::WorldPosition2D(Vector2::new(hit_enu.x, hit_enu.y));

    goal_events.write(GoalCommandEvent {
        agent: agent_entity,
        goal: goal.clone(),
    });
    goal_registry.0.insert(agent_entity, goal);

    info!(
        "[Interaction] Goal → ENU ({:.1}, {:.1}) for {:?}",
        hit_enu.x, hit_enu.y, agent_entity
    );
}
