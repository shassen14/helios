// helios_sim/src/simulation/plugins/planning/mod.rs
//
// PlanningPlugin stub. Will be wired to AutonomyPipelineComponent in a later step.

pub mod interaction;

use bevy::prelude::*;

use crate::prelude::AppState;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::events::GoalCommandEvent;
use interaction::{GoalRegistry, SelectedAgent};

pub struct PlanningPlugin;

impl Plugin for PlanningPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SelectedAgent>()
            .init_resource::<GoalRegistry>()
            .add_event::<GoalCommandEvent>()
            .add_systems(
                FixedUpdate,
                planning_stub
                    .in_set(SimulationSet::Planning)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(
                Update,
                (
                    interaction::agent_select_system,
                    interaction::click_goal_system,
                )
                    .run_if(in_state(AppState::Running)),
            );
    }
}

/// No-op stub. Planning will be wired to the pipeline in a later step.
fn planning_stub() {}
