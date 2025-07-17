// helios_sim/src/simulation/core/components.rs

use bevy::prelude::Component;
use helios_core::prelude::{EstimationDynamics, Measurement}; // Import the pure traits

// --- Wrapper Components for Core Traits ---

/// A Bevy component that wraps a pure `Dynamics` trait object.
/// This is attached to an agent entity to define its physical model.
#[derive(Component)]
pub struct EstimationDynamicsModel(pub Box<dyn EstimationDynamics>);

/// A Bevy component that wraps a pure `Measurement` trait object.
/// This is attached to a sensor entity to define its mathematical model.
#[derive(Component)]
pub struct MeasurementModel(pub Box<dyn Measurement>);

// As you create more core traits (e.g., for planners, controllers),
// you would add their corresponding Bevy wrapper components here.
// #[derive(Component)]
// pub struct PlannerModel(pub Box<dyn Planner>);
