// helios_sim/src/simulation/plugins/isolation/mock_path.rs
//
// MockPathInjectorPlugin: reads `simulation.mock_path` from the scenario TOML,
// builds a `helios_core::planning::Path`, and injects it into every agent's
// `PathFollowingComponent`.  The PathFollowingPlugin then runs unchanged —
// the path is already set when the first tick runs.

use bevy::prelude::*;
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
use helios_core::planning::types::Path;
use helios_core::types::TrajectoryPoint;
use helios_runtime::stage::PipelineLevel;
use serde::Deserialize;

use crate::prelude::AppState;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::plugins::autonomy::PathFollowingComponent;

pub struct MockPathInjectorPlugin;

impl Plugin for MockPathInjectorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(AppState::Running), inject_mock_path);
    }
}

// ---------------------------------------------------------------------------
// Fixture TOML format
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct PathFixture {
    #[serde(default = "default_level")]
    level: String,
    waypoints: Vec<WaypointFixture>,
}

#[derive(Deserialize)]
struct WaypointFixture {
    /// ENU world position [x, y, z] in metres.
    position: [f64; 3],
    /// Heading in degrees (0 = East, 90 = North).
    #[serde(default)]
    yaw_deg: f64,
}

fn default_level() -> String {
    "local".to_string()
}

// ---------------------------------------------------------------------------
// System
// ---------------------------------------------------------------------------

fn inject_mock_path(
    scenario: Res<ScenarioConfig>,
    mut query: Query<Option<&mut PathFollowingComponent>>,
) {
    let Some(ref path_file) = scenario.simulation.mock_path else {
        return;
    };

    let fixture: PathFixture = match load_fixture(path_file) {
        Ok(f) => f,
        Err(e) => {
            error!("[MockPathInjector] Failed to load '{}': {}", path_file, e);
            return;
        }
    };

    let level = match fixture.level.as_str() {
        "global" => PipelineLevel::Global,
        _ => PipelineLevel::Local,
    };

    let path = build_path(&fixture, &level);
    let path_len = path.len();

    for pf_opt in &mut query {
        if let Some(mut pf) = pf_opt {
            pf.0.set_path(path.clone());
        }
    }

    info!(
        "[MockPathInjector] Injected {} waypoints from '{}' at level {:?}",
        path_len, path_file, level
    );
}

fn load_fixture(path: &str) -> Result<PathFixture, String> {
    let content = std::fs::read_to_string(path).map_err(|e| format!("read error: {e}"))?;
    toml::from_str(&content).map_err(|e| format!("parse error: {e}"))
}

fn build_path(fixture: &PathFixture, level: &PipelineLevel) -> Path {
    let waypoints: Vec<TrajectoryPoint> = fixture
        .waypoints
        .iter()
        .enumerate()
        .map(|(i, wp)| {
            let layout = vec![
                StateVariable::Px(FrameId::World),
                StateVariable::Py(FrameId::World),
                StateVariable::Pz(FrameId::World),
                StateVariable::Qx(FrameId::World, FrameId::World),
                StateVariable::Qy(FrameId::World, FrameId::World),
                StateVariable::Qz(FrameId::World, FrameId::World),
                StateVariable::Qw(FrameId::World, FrameId::World),
            ];
            let mut state = FrameAwareState::new(layout, 1e-6, 0.0);
            state.vector[0] = wp.position[0];
            state.vector[1] = wp.position[1];
            state.vector[2] = wp.position[2];
            // Build orientation from yaw.
            let half_yaw = wp.yaw_deg.to_radians() * 0.5;
            state.vector[3] = 0.0; // Qx
            state.vector[4] = 0.0; // Qy
            state.vector[5] = half_yaw.sin(); // Qz
            state.vector[6] = half_yaw.cos(); // Qw
            TrajectoryPoint {
                state,
                state_dot: None,
                time: i as f64,
            }
        })
        .collect();

    let level_key = match level {
        PipelineLevel::Global => "global".to_string(),
        PipelineLevel::Local => "local".to_string(),
        PipelineLevel::Custom(s) => s.clone(),
    };

    Path {
        waypoints,
        timestamp: 0.0,
        level_key,
    }
}
