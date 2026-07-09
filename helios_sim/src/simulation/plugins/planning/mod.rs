// helios_sim/src/simulation/plugins/planning/mod.rs
//
// PlanningPlugin: forwards GoalCommandEvents into each agent's
// AutonomyPipeline via `inject_mission_goal`. The actual planner runs as a
// PipelineNode inside the DAG; this system is the host→pipeline bridge for
// the long-lived mission goal slot.

use bevy::prelude::*;

use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::components::GoalDispatched;
use crate::simulation::core::events::GoalCommandEvent;
use crate::simulation::core::sim_runtime::SimRuntime;
use crate::simulation::core::transforms::TfTree;
use crate::simulation::plugins::autonomy::AutonomyPipelineComponent;
use crate::{prelude::AppState, simulation::core::components::ConfiguredMissionGoal};

pub struct PlanningPlugin;

impl Plugin for PlanningPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<GoalCommandEvent>().add_systems(
            FixedUpdate,
            // Both are host→pipeline goal glue, so they share the BrainInput set.
            // The chain is load-bearing: `dispatch_configured_goals` writes a
            // `GoalCommandEvent` that `forward_goal_events` reads the same tick,
            // and a Bevy message is only readable this tick if the writer ran first.
            (dispatch_configured_goals, forward_goal_events)
                .chain()
                .in_set(SimulationSet::BrainInput)
                .run_if(in_state(AppState::Running)),
        );
    }
}

/// Drains pending `GoalCommandEvent`s and writes each into the target agent's
/// pipeline mission slot. The mission slot is a host-state channel, so the
/// value persists across ticks until overwritten.
fn forward_goal_events(
    mut events: MessageReader<GoalCommandEvent>,
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

/// Auto-starts each agent's mission. On the first tick an agent is seen, this
/// synthesizes the `GoalCommandEvent` a mouse click would have produced, taken
/// from the agent's authored `goal_pose` — so a headless run drives toward its
/// goal with no user input. `forward_goal_events` (chained after this in
/// `SimulationSet::BrainInput`) consumes the event and injects it into the pipeline.
///
/// The `GoalDispatched` marker, stamped once here, drops the agent from the
/// query on every later tick: the authored goal is sent exactly once, so a
/// mid-run click can retarget the agent without this system stomping it back.
fn dispatch_configured_goals(
    agents: Query<(Entity, &ConfiguredMissionGoal), Without<GoalDispatched>>,
    mut goal_events: MessageWriter<GoalCommandEvent>,
    mut commands: Commands,
) {
    for (entity, goal) in agents {
        let event = GoalCommandEvent {
            agent: entity,
            goal: goal.0.clone(),
        };

        goal_events.write(event);

        commands.entity(entity).insert(GoalDispatched);
    }
}
