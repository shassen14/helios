// helios_sim/src/simulation/plugins/vehicles/ackermann/mod.rs

pub mod adapter;
pub mod components;
pub(super) mod systems;

pub use adapter::{AckermannAdapterComponent, AckermannOutputAdapter, DefaultAckermannAdapter};
pub use components::{AckermannActuator, AckermannCommand, AckermannParameters};

use crate::simulation::core::app_state::{AppState, SceneBuildSet, SimulationSet};
use bevy::prelude::*;
use systems::{
    attach_ackermann_physics, drive_ackermann_cars, process_ackermann_logic, setup_ackermann_assets,
};

/// Shared mesh/material handles — created once at scene setup, reused for every car.
#[derive(Resource)]
pub(super) struct AckermannAssets {
    pub(super) body_mesh: Handle<Mesh>,
    pub(super) body_material: Handle<StandardMaterial>,
    pub(super) wheel_mesh: Handle<Mesh>,
    pub(super) wheel_material: Handle<StandardMaterial>,
}

pub struct AckermannCarPlugin;

impl Plugin for AckermannCarPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(AppState::SceneBuilding), setup_ackermann_assets)
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    process_ackermann_logic.in_set(SceneBuildSet::ProcessVehicle),
                    attach_ackermann_physics.in_set(SceneBuildSet::Physics),
                ),
            )
            .add_systems(
                FixedUpdate,
                drive_ackermann_cars.in_set(SimulationSet::Actuation),
            );
    }
}
