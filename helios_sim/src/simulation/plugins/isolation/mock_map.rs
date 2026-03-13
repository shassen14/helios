// helios_sim/src/simulation/plugins/isolation/mock_map.rs
//
// MockMapInjectorPlugin: reads `simulation.mock_map` from the scenario TOML and
// replaces every agent's `MapperComponent` with a `StaticMapProvider` built from
// the fixture dimensions.

use bevy::prelude::*;
use nalgebra::Isometry3;
use serde::Deserialize;

use crate::prelude::AppState;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::plugins::autonomy::MapperComponent;
use helios_runtime::mapping::StaticMapProvider;
use helios_runtime::stage::PipelineLevel;

pub struct MockMapInjectorPlugin;

impl Plugin for MockMapInjectorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(AppState::Running), inject_mock_map);
    }
}

// ---------------------------------------------------------------------------
// Fixture TOML format
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct MapFixture {
    width_m: f64,
    height_m: f64,
    resolution_m: f64,
    /// ENU origin [x, y] in metres.
    #[serde(default)]
    origin: [f64; 2],
}

// ---------------------------------------------------------------------------
// System
// ---------------------------------------------------------------------------

fn inject_mock_map(
    mut commands: Commands,
    scenario: Res<ScenarioConfig>,
    query: Query<Entity, With<MapperComponent>>,
) {
    let Some(ref map_file) = scenario.simulation.mock_map else {
        return;
    };

    let fixture: MapFixture = match load_fixture(map_file) {
        Ok(f) => f,
        Err(e) => {
            error!("[MockMapInjector] Failed to load '{}': {}", map_file, e);
            return;
        }
    };

    let origin = Isometry3::translation(fixture.origin[0], fixture.origin[1], 0.0);
    let static_map = StaticMapProvider::from_fixture(
        fixture.width_m,
        fixture.height_m,
        fixture.resolution_m,
        origin,
        PipelineLevel::Local,
    );

    for entity in &query {
        commands
            .entity(entity)
            .insert(MapperComponent(Box::new(static_map.clone())));
    }

    info!(
        "[MockMapInjector] Replaced mapper with {}×{} m static map ({}m/cell)",
        fixture.width_m, fixture.height_m, fixture.resolution_m
    );
}

fn load_fixture(path: &str) -> Result<MapFixture, String> {
    let content = std::fs::read_to_string(path).map_err(|e| format!("read error: {e}"))?;
    toml::from_str(&content).map_err(|e| format!("parse error: {e}"))
}
