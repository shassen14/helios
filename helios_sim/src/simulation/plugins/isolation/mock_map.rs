// helios_sim/src/simulation/plugins/isolation/mock_map.rs
//
// MockMapInjectorPlugin: reads `simulation.mock_map` from the scenario TOML and
// primes every agent's `MapperComponent` with a pose-update so the planner
// finds a populated map on its first tick.

use bevy::prelude::*;
use nalgebra::Isometry3;
use serde::Deserialize;

use crate::prelude::AppState;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::plugins::autonomy::MapperComponent;
use helios_core::estimation::FilterContext;
use helios_core::messages::ModuleInput;
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
    #[allow(dead_code)]
    width_m: f64,
    #[allow(dead_code)]
    height_m: f64,
    #[allow(dead_code)]
    resolution_m: f64,
    /// ENU origin [x, y] in metres.
    #[serde(default)]
    origin: [f64; 2],
}

// ---------------------------------------------------------------------------
// System
// ---------------------------------------------------------------------------

fn inject_mock_map(
    scenario: Res<ScenarioConfig>,
    mut query: Query<&mut MapperComponent>,
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
    let context = FilterContext { tf: None };

    for mut mapper_comp in &mut query {
        for lm in &mut mapper_comp.0.mappers {
            if lm.level == PipelineLevel::Local || lm.level == PipelineLevel::Global {
                // Prime the mapper with a pose update so its internal cache
                // is initialised.  The map will be all-unknown (log-odds 0),
                // which the planner treats as traversable.
                lm.mapper
                    .process(&ModuleInput::PoseUpdate { pose: origin }, &context);
            }
        }
    }

    info!("[MockMapInjector] Primed mapper from '{}'", map_file);
}

fn load_fixture(path: &str) -> Result<MapFixture, String> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| format!("read error: {e}"))?;
    toml::from_str(&content).map_err(|e| format!("parse error: {e}"))
}
