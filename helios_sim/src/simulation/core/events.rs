// helios_sim/src/simulation/core/events.rs
use bevy::prelude::{Entity, Event};
use helios_core::messages::MeasurementMessage;
use helios_core::planning::types::PlannerGoal;

/// Bevy wrapper for a raw sensor measurement.
#[derive(Event, Clone)]
pub struct BevyMeasurementMessage(pub MeasurementMessage);

/// Commands a navigation goal change for a specific agent.
/// Send this event from any system (UI, scenario logic, test harness)
/// to update the active planner goal mid-simulation.
#[derive(Event)]
pub struct GoalCommandEvent {
    pub agent: Entity,
    pub goal: PlannerGoal,
}
