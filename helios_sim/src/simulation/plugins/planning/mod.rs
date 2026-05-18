// helios_sim/src/simulation/plugins/planning/mod.rs
//
// PlanningPlugin: forwards GoalCommandEvents into each agent's
// AutonomyPipeline via `inject_mission_goal`. The actual planner runs as a
// PipelineNode inside the DAG; this system is the host→pipeline bridge for
// the long-lived mission goal slot.

pub mod gizmos;
pub mod interaction;

use bevy::prelude::*;
use std::time::Duration;

use crate::prelude::AppState;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::events::GoalCommandEvent;
use crate::simulation::core::sim_runtime::SimRuntime;
use crate::simulation::core::transforms::TfTree;
use crate::simulation::plugins::autonomy::AutonomyPipelineComponent;
use interaction::{GoalRegistry, SelectedAgent};

pub struct PlanningPlugin;

#[derive(Resource)]
struct ChainStatusTimer(Timer);

impl Plugin for PlanningPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SelectedAgent>()
            .init_resource::<GoalRegistry>()
            .add_event::<GoalCommandEvent>()
            .insert_resource(ChainStatusTimer(Timer::new(
                Duration::from_secs(1),
                TimerMode::Repeating,
            )))
            .add_systems(
                FixedUpdate,
                forward_goal_events
                    .in_set(SimulationSet::Planning)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(
                Update,
                (
                    interaction::agent_select_system,
                    interaction::click_goal_system,
                    gizmos::draw_selection,
                    log_chain_status,
                )
                    .run_if(in_state(AppState::Running)),
            );
    }
}

/// Once per second, logs which links in the goal → path → control chain are
/// populated for the selected agent. Lets you see in one glance where the
/// chain breaks during debugging.
fn log_chain_status(
    time: Res<Time>,
    mut timer: ResMut<ChainStatusTimer>,
    selected: Res<SelectedAgent>,
    pipelines: Query<&AutonomyPipelineComponent>,
) {
    if !timer.0.tick(time.delta()).just_finished() {
        return;
    }
    let Some(entity) = selected.0 else {
        return;
    };
    let Ok(pipeline) = pipelines.get(entity) else {
        return;
    };
    let p = &pipeline.0;
    let map_kind = match p.read_any_map().as_deref() {
        None => ".",
        Some(s) => match &s.value {
            helios_core::mapping::MapData::OccupancyGrid2D { .. } => "Occ2D",
            helios_core::mapping::MapData::FeatureMap { .. } => "Feat",
        },
    };
    let scan_present = p
        .bus()
        .read::<Vec<helios_core::data::sensor::SensorReading<helios_core::data::sensor::PointCloud2D>>>(
            helios_runtime::port::ChannelKey::of::<Vec<helios_core::data::sensor::SensorReading<helios_core::data::sensor::PointCloud2D>>>(),
        )
        .is_some();
    info!(
        "[Chain] state={} scan={} map={} goal={} path={} control={}",
        if p.read_state().is_some() { "Y" } else { "." },
        if scan_present { "Y" } else { "." },
        map_kind,
        if p.read_mission_goal().is_some() { "Y" } else { "." },
        if p.read_any_path().is_some() { "Y" } else { "." },
        if p.read_control().is_some() { "Y" } else { "." },
    );
}

/// Drains pending `GoalCommandEvent`s and writes each into the target agent's
/// pipeline mission slot. The mission slot is a host-state channel, so the
/// value persists across ticks until overwritten.
fn forward_goal_events(
    mut events: EventReader<GoalCommandEvent>,
    pipelines: Query<&AutonomyPipelineComponent>,
    tf_tree: Res<TfTree>,
    time: Res<Time>,
) {
    if events.is_empty() {
        return;
    }

    let runtime = SimRuntime {
        tf: &tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    for event in events.read() {
        let Ok(pipeline) = pipelines.get(event.agent) else {
            warn!(
                "[Planning] GoalCommandEvent for {:?} but agent has no AutonomyPipelineComponent",
                event.agent
            );
            continue;
        };
        pipeline.0.inject_mission_goal(event.goal.clone(), &runtime);
    }
}
