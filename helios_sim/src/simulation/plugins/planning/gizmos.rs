// helios_sim/src/simulation/plugins/planning/gizmos.rs
//
// Draws the selected-agent ring and goal marker in Bevy world space.
//
// Coordinate convention (ENU → Bevy):
//   bevy_x = enu_x,  bevy_y = enu_z (≈0 on ground),  bevy_z = -enu_y

use bevy::prelude::*;
use helios_core::planning::types::PlannerGoal;

use super::interaction::{GoalRegistry, SelectedAgent};
use crate::simulation::plugins::autonomy::ControlPipelineComponent;

const COLOR_SELECTION: Color = Color::srgb(0.0, 1.0, 0.0); // green
const COLOR_GOAL: Color = Color::srgb(1.0, 0.0, 1.0); // magenta

const SELECTION_RING_RADIUS: f32 = 3.0;
const GOAL_SPHERE_RADIUS: f32 = 0.8;

/// ENU 2D goal position → Bevy Vec3 at a slight hover above the ground.
fn goal_to_bevy(goal: &PlannerGoal) -> Option<Vec3> {
    match goal {
        PlannerGoal::WorldPosition2D(v) => Some(Vec3::new(v.x as f32, 0.2, -(v.y as f32))),
        PlannerGoal::WorldPose(iso) => Some(Vec3::new(
            iso.translation.x as f32,
            0.2,
            -(iso.translation.y as f32),
        )),
        PlannerGoal::GlobalPathWaypoint { .. } => None,
    }
}

/// Draws a green ring under the selected agent and a magenta sphere + line at its goal.
pub fn draw_selection(
    selected: Res<SelectedAgent>,
    goal_registry: Res<GoalRegistry>,
    agents: Query<&GlobalTransform, With<ControlPipelineComponent>>,
    mut gizmos: Gizmos,
) {
    let Some(entity) = selected.0 else { return };
    let Ok(gt) = agents.get(entity) else { return };

    // Green ring on the ground plane under the agent.
    let pos = gt.translation();
    let ring_centre = Vec3::new(pos.x, 0.05, pos.z);
    // Rotate so the circle lies in the XZ plane (horizontal).
    let iso = Isometry3d::new(
        ring_centre,
        Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
    );
    gizmos.circle(iso, SELECTION_RING_RADIUS, COLOR_SELECTION);

    // Magenta goal sphere + connecting line.
    if let Some(goal) = goal_registry.0.get(&entity) {
        if let Some(goal_pos) = goal_to_bevy(goal) {
            gizmos.sphere(
                Isometry3d::from_translation(goal_pos),
                GOAL_SPHERE_RADIUS,
                COLOR_GOAL,
            );
            gizmos.line(ring_centre, goal_pos, COLOR_GOAL);
        }
    }
}
