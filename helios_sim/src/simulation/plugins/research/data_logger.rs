// helios_sim/src/simulation/plugins/research/data_logger.rs
//
// DataLoggerPlugin: samples ground truth + estimated state + control output
// each FixedUpdate tick.  Flushes to CSV on simulation exit.

use bevy::prelude::*;
use std::fmt::Write as FmtWrite;

use crate::prelude::AppState;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::components::{ControlOutputComponent, GroundTruthState};
use crate::simulation::plugins::autonomy::EstimatorComponent;

// ---------------------------------------------------------------------------
// Resource
// ---------------------------------------------------------------------------

/// In-memory sample buffer.
#[derive(Resource, Default)]
pub struct DataLogBuffer {
    pub rows: Vec<LogRow>,
}

#[derive(Default, Clone, Debug)]
pub struct LogRow {
    pub timestamp_s: f64,
    pub gt_x: f64,
    pub gt_y: f64,
    pub gt_yaw: f64,
    pub est_x: f64,
    pub est_y: f64,
    pub est_yaw: f64,
    pub ctrl_steer: f64,
    pub ctrl_speed: f64,
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

pub struct DataLoggerPlugin;

impl Plugin for DataLoggerPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DataLogBuffer>()
            .add_systems(
                FixedUpdate,
                sample_data
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(OnExit(AppState::Running), flush_log);
    }
}

// ---------------------------------------------------------------------------
// Systems
// ---------------------------------------------------------------------------

fn sample_data(
    time: Res<Time>,
    query: Query<(
        &GroundTruthState,
        Option<&EstimatorComponent>,
        Option<&ControlOutputComponent>,
    )>,
    mut buffer: ResMut<DataLogBuffer>,
) {
    let t = time.elapsed_secs_f64();

    for (gt, est_opt, ctrl_opt) in &query {
        let gt_yaw = gt.pose.rotation.euler_angles().2;

        let (est_x, est_y, est_yaw): (f64, f64, f64) = est_opt
            .and_then(|e| e.0.get_state())
            .and_then(|s| s.get_pose_isometry())
            .map(|iso| {
                let yaw = iso.rotation.euler_angles().2;
                (iso.translation.x, iso.translation.y, yaw)
            })
            .unwrap_or((gt.pose.translation.x, gt.pose.translation.y, gt_yaw));

        let (ctrl_steer, ctrl_speed): (f64, f64) = ctrl_opt
            .map(|c| extract_actuators(&c.0))
            .unwrap_or((0.0, 0.0));

        buffer.rows.push(LogRow {
            timestamp_s: t,
            gt_x: gt.pose.translation.x,
            gt_y: gt.pose.translation.y,
            gt_yaw,
            est_x,
            est_y,
            est_yaw,
            ctrl_steer,
            ctrl_speed,
        });
    }
}

fn extract_actuators(output: &helios_core::control::ControlOutput) -> (f64, f64) {
    use helios_core::control::ControlOutput;
    match output {
        ControlOutput::RawActuators(v) => {
            let speed = v.first().copied().unwrap_or(0.0);
            let steer = v.get(1).copied().unwrap_or(0.0);
            (steer, speed)
        }
        _ => (0.0, 0.0),
    }
}

fn flush_log(buffer: Res<DataLogBuffer>, scenario: Res<ScenarioConfig>) {
    if buffer.rows.is_empty() {
        return;
    }

    let path = scenario
        .metrics
        .output_path
        .as_deref()
        .unwrap_or("results/data_log.csv");

    let mut out =
        String::from("timestamp_s,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,ctrl_steer,ctrl_speed\n");
    for row in &buffer.rows {
        let _ = writeln!(
            out,
            "{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            row.timestamp_s,
            row.gt_x,
            row.gt_y,
            row.gt_yaw,
            row.est_x,
            row.est_y,
            row.est_yaw,
            row.ctrl_steer,
            row.ctrl_speed,
        );
    }

    // Ensure parent directory exists.
    if let Some(parent) = std::path::Path::new(path).parent() {
        let _ = std::fs::create_dir_all(parent);
    }

    if let Err(e) = std::fs::write(path, out) {
        error!("[DataLogger] Failed to write '{}': {}", path, e);
    } else {
        info!(
            "[DataLogger] {} rows written to '{}'",
            buffer.rows.len(),
            path
        );
    }
}
