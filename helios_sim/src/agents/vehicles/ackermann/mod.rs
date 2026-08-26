pub(super) mod systems;

use crate::core::app_state::{AppState, SceneBuildSet, SimulationSet};

use bevy::prelude::*;
use systems::{attach_ackermann_visual, drive_ackermann_cars, setup_ackermann_assets};

/// Shared mesh/material handles — created once at scene setup, reused for every car.
#[derive(Resource)]
pub(super) struct AckermannAssets {
    pub(super) body_mesh: Handle<Mesh>,
    pub(super) body_material: Handle<StandardMaterial>,
    pub(super) wheel_mesh: Handle<Mesh>,
    pub(super) wheel_material: Handle<StandardMaterial>,
}

/// The L0 car family: shared visual assets, the residual cosmetic visual, and the
/// L0 plant apply. Body construction (topology/plant/collision) is the generic
/// `build_embodiment` dispatch in `HeliosVehiclesPlugin`, keyed on config `kind`s
/// this family registers; this plugin owns only what is still car-specific.
pub struct AckermannCarPlugin;

impl Plugin for AckermannCarPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(AppState::SceneBuilding), setup_ackermann_assets)
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                attach_ackermann_visual.in_set(SceneBuildSet::Physics),
            )
            .add_systems(
                FixedUpdate,
                drive_ackermann_cars.in_set(SimulationSet::Actuation),
            );
    }
}
