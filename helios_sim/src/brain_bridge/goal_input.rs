//! Ingress: feeds host mission intent into each agent's pipeline.
//!
//! Two chained systems in `SimulationSet::BrainInput`: `dispatch_configured_goals`
//! synthesizes the authored goal once per agent, then `forward_goal_events`
//! publishes any pending `GoalCommandEvent` to the goal channels the agent's
//! planners read, via `HostInputPublisher`. The planner itself is a DAG node;
//! this is only the host→pipeline goal bridge.

use crate::brain_bridge::components::MissionGoalChannels;
use crate::core::components::ConfiguredMissionGoal;
use crate::core::components::GoalDispatched;
use crate::core::events::GoalCommandEvent;
use crate::prelude::HostInputPublisher;

use helios_core::data::MonotonicTime;
use helios_core::prelude::PlannerGoal;
use helios_runtime::port::InternalChannel;

use bevy::prelude::*;

/// Drains pending `GoalCommandEvent`s and publishes each to every goal channel
/// the target agent declared in [`MissionGoalChannels`], via [`HostInputPublisher`].
///
/// One goal fans out to all of the agent's planner channels, stamped at the
/// current tick clock. The value lands on a last-known-good bus channel, so it
/// persists across ticks until the next goal overwrites it. An event for an
/// agent with no [`MissionGoalChannels`] — a plannerless agent, or one whose
/// pipeline failed to build — has nowhere to go and is dropped with a warning.
pub fn forward_goal_events(
    mut events: MessageReader<GoalCommandEvent>,
    mut host_inputs: HostInputPublisher,
    goal_channels: Query<&MissionGoalChannels>,
    time: Res<Time>,
) {
    let timestamp = MonotonicTime(time.elapsed_secs_f64());

    for event in events.read() {
        let Ok(channels) = goal_channels.get(event.agent) else {
            warn!(
                "[Planning] GoalCommandEvent for {:?} but agent has no MissionGoalChannels",
                event.agent
            );
            continue;
        };

        for channel in &channels.0 {
            host_inputs.publish(
                event.agent,
                InternalChannel::named::<PlannerGoal>(channel.as_str()),
                event.goal.clone(),
                timestamp,
            );
        }
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
pub fn dispatch_configured_goals(
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

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::planning::types::PlannerGoal;
    use nalgebra::Vector2;

    /// Counts `GoalCommandEvent`s seen across ticks. Chained after the system
    /// under test, so its cursor sees exactly the events dispatch just wrote.
    #[derive(Resource, Default)]
    struct Emitted(usize);

    fn count_emitted(mut reader: MessageReader<GoalCommandEvent>, mut n: ResMut<Emitted>) {
        n.0 += reader.read().count();
    }

    /// `dispatch_configured_goals` sends each agent's authored goal exactly once
    /// and stamps `GoalDispatched` so a later tick never re-sends it — the
    /// invariant that lets a mid-run retarget survive without being stomped.
    #[test]
    fn dispatch_emits_once_then_marks_dispatched() {
        let mut app = App::new();
        app.add_message::<GoalCommandEvent>()
            .init_resource::<Emitted>()
            .add_systems(Update, (dispatch_configured_goals, count_emitted).chain());

        let goal = PlannerGoal::WorldPosition2D(Vector2::new(1.0, 2.0));
        let a = app
            .world_mut()
            .spawn(ConfiguredMissionGoal(goal.clone()))
            .id();
        let b = app.world_mut().spawn(ConfiguredMissionGoal(goal)).id();

        // First tick: one event per agent, both now marked dispatched.
        app.world_mut().run_schedule(Update);
        assert_eq!(app.world().resource::<Emitted>().0, 2);
        assert!(app.world().get::<GoalDispatched>(a).is_some());
        assert!(app.world().get::<GoalDispatched>(b).is_some());

        // Second tick: GoalDispatched excludes both from the query — no re-send.
        app.world_mut().run_schedule(Update);
        assert_eq!(app.world().resource::<Emitted>().0, 2);
    }
}
