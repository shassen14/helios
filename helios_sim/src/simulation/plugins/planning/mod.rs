// helios_sim/src/simulation/plugins/planning/mod.rs
//
// PlanningPlugin: drives all planner stages each tick and handles GoalCommandEvents.

pub mod interaction;
mod gizmos;

use std::collections::HashMap;

use bevy::prelude::*;
use helios_core::mapping::MapData;
use helios_runtime::stage::PipelineLevel;

use crate::prelude::AppState;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::events::GoalCommandEvent;
use crate::simulation::core::sim_runtime::SimRuntime;
use crate::simulation::core::transforms::TfTree;
use crate::simulation::plugins::autonomy::{
    ControlPipelineComponent, EstimatorComponent, MapperComponent,
};
use interaction::{GoalRegistry, SelectedAgent};

pub struct PlanningPlugin;

impl Plugin for PlanningPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SelectedAgent>()
            .init_resource::<GoalRegistry>()
            .add_event::<GoalCommandEvent>()
            .add_systems(
                FixedUpdate,
                planning_system
                    .in_set(SimulationSet::Planning)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(
                FixedUpdate,
                goal_command_system
                    .in_set(SimulationSet::Behavior)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(
                Update,
                (
                    interaction::agent_select_system,
                    interaction::click_goal_system,
                    gizmos::draw_selection,
                )
                    .run_if(in_state(AppState::Running)),
            );
    }
}

/// Runs all planners for every agent each tick.
///
/// Builds a map-by-level lookup from EstimatorComponent (SLAM global map) and
/// MapperComponent (local / global mappers), then calls `step_planners`.
fn planning_system(
    time: Res<Time>,
    tf_tree: Res<TfTree>,
    mut query: Query<(
        &EstimatorComponent,
        &MapperComponent,
        &mut ControlPipelineComponent,
    )>,
) {
    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    for (estimator, mapper, mut control) in &mut query {
        let Some(state) = estimator.0.get_state() else {
            continue;
        };

        // Build map lookup: prefer SLAM global map over global mapper.
        let mut maps: HashMap<PipelineLevel, &MapData> = HashMap::new();

        if let Some(slam_map) = estimator.0.get_slam_map() {
            if !matches!(*slam_map, MapData::None) {
                maps.insert(PipelineLevel::Global, slam_map);
            }
        }
        if maps.get(&PipelineLevel::Global).is_none() {
            if let Some(global_map) = mapper.0.get_map(&PipelineLevel::Global) {
                if !matches!(*global_map, MapData::None) {
                    maps.insert(PipelineLevel::Global, global_map);
                }
            }
        }
        if let Some(local_map) = mapper.0.get_map(&PipelineLevel::Local) {
            if !matches!(*local_map, MapData::None) {
                maps.insert(PipelineLevel::Local, local_map);
            }
        }

        control.0.step_planners(state, &maps, time.elapsed_secs_f64(), &runtime);
    }
}

/// Processes `GoalCommandEvent`s: finds the target agent and calls `set_goal`.
/// Also keeps `GoalRegistry` in sync so gizmos and UI reflect the latest goal per agent.
fn goal_command_system(
    mut events: EventReader<GoalCommandEvent>,
    mut query: Query<&mut ControlPipelineComponent>,
    mut goal_registry: ResMut<GoalRegistry>,
) {
    for event in events.read() {
        if let Ok(mut control) = query.get_mut(event.agent) {
            control.0.set_goal(event.goal.clone());
        }
        goal_registry.0.insert(event.agent, event.goal.clone());
    }
}
