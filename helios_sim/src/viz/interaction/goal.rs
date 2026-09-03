//! Right-click-to-retarget: a click on the world hands the selected agent a new
//! mission goal — the same [`GoalCommandEvent`] the authored `goal_pose` produces
//! at startup. A mouse click and a config goal travel the identical host→pipeline
//! path (`forward_goal_events` in `SimulationSet::BrainInput`), so this adds no
//! pipeline method and no new channel: the goal rides the bus like any host input.
//!
//! Left-click is selection ([`Selected`], via `selection::on_click_select`);
//! right-click is "go there", so the two verbs never contend for one button. The
//! goal targets whichever agent is [`Selected`] and can receive one — select an
//! agent, then right-click its destination. Added only by `helios_play`, never the
//! headless host, which drives from the authored goal via `dispatch_configured_goals`.

use crate::{
    brain_bridge::components::MissionGoalChannels,
    core::{events::GoalCommandEvent, transforms::EnuVector},
    viz::interaction::selection::Selected,
};

use bevy::prelude::*;
use helios_core::prelude::PlannerGoal;
use nalgebra::Vector2;

/// Registers the right-click goal observer. Relies on the mesh-picking backend
/// that `SelectionPlugin` installs, so it is added after that plugin and never
/// adds `MeshPickingPlugin` itself — Bevy panics on a duplicate plugin.
pub struct GoalPickingPlugin;

impl Plugin for GoalPickingPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(on_click_goal);
    }
}

/// Global click observer: on a right-click that landed on a world mesh, converts
/// the hit point to an ENU 2D goal and emits a [`GoalCommandEvent`] for the
/// selected agent. Silently no-ops when the click was not a right-click, the
/// picking backend reported no hit position, or no goal-taking agent is selected
/// — a right-click on empty space, or with nothing selected, is not an error.
///
/// The hit arrives in Bevy world space; [`EnuVector`] is the one conversion to
/// ENU, keeping the axis swap inside `core/transforms`. Only x and y reach the
/// goal — the planner is 2D, so a click's height is discarded until the goal
/// carries a `WorldPose`.
pub fn on_click_goal(
    mut click: On<Pointer<Click>>,
    selected: Query<Entity, (With<Selected>, With<MissionGoalChannels>)>,
    mut goals: MessageWriter<GoalCommandEvent>,
) {
    click.propagate(false);
    if click.event.button != PointerButton::Secondary {
        return;
    }
    let Some(hit) = click.event.hit.position else {
        return;
    };
    let Ok(agent) = selected.single() else {
        return;
    };

    let enu = EnuVector::from(hit).0;
    let goal = PlannerGoal::WorldPosition2D(Vector2::new(enu.x, enu.y));
    goals.write(GoalCommandEvent { agent, goal });
}
