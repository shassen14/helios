// helios_sim/src/simulation/plugins/metrics/control_metrics.rs
//
// ControlMetrics resource + systems.
// Pure-data accumulation — all math delegates to helios_core::control::metrics.

use bevy::prelude::*;
use helios_core::control::metrics as core_metrics;
use nalgebra::Vector2;

use crate::prelude::AppState;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::core::app_state::SimulationSet;
use crate::simulation::core::components::GroundTruthState;
use crate::simulation::plugins::autonomy::PathFollowingOutputComponent;

// ---------------------------------------------------------------------------
// Resource
// ---------------------------------------------------------------------------

/// Accumulated samples and computed metrics for one simulation run.
#[derive(Resource, Default)]
pub struct ControlMetrics {
    /// (timestamp_s, forward_speed_mps)
    pub speed_history: Vec<(f64, f64)>,
    /// Actual 2-D positions in ENU.
    pub actual_positions: Vec<Vector2<f64>>,
    /// Reference 2-D positions (from cached path lookahead).
    pub reference_positions: Vec<Vector2<f64>>,
    /// Actual headings (radians, ENU yaw).
    pub actual_headings: Vec<f64>,
    /// Reference headings (radians).
    pub reference_headings: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

pub struct ControlMetricsPlugin;

impl Plugin for ControlMetricsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ControlMetrics>()
            .add_systems(
                FixedUpdate,
                sample_metrics
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(OnEnter(AppState::Flushing), print_metrics);
    }
}

// ---------------------------------------------------------------------------
// Systems
// ---------------------------------------------------------------------------

/// Samples ground-truth state and active path waypoint each FixedUpdate tick.
fn sample_metrics(
    time: Res<Time>,
    gt_query: Query<(&GroundTruthState, &PathFollowingOutputComponent)>,
    mut metrics: ResMut<ControlMetrics>,
) {
    let t = time.elapsed_secs_f64();

    for (gt, pf_output) in &gt_query {
        let speed = gt.linear_velocity.norm();
        metrics.speed_history.push((t, speed));

        let pos2d = Vector2::new(gt.pose.translation.x, gt.pose.translation.y);
        metrics.actual_positions.push(pos2d);

        let yaw = gt.pose.rotation.euler_angles().2;
        metrics.actual_headings.push(yaw);

        // Reference from active path following waypoint.
        if let Some(wp) = &pf_output.0 {
            let ref_x = wp.state.vector[0];
            let ref_y = wp.state.vector[1];
            metrics.reference_positions.push(Vector2::new(ref_x, ref_y));

            // Heading from Qz/Qw in the waypoint state (simplified yaw extraction).
            let qz = wp.state.vector.get(5).copied().unwrap_or(0.0);
            let qw = wp.state.vector.get(6).copied().unwrap_or(1.0);
            let ref_yaw = 2.0 * qz.atan2(qw);
            metrics.reference_headings.push(ref_yaw);
        } else {
            metrics.reference_positions.push(pos2d);
            metrics.reference_headings.push(yaw);
        }
    }
}

/// Prints the metrics table on simulation exit.
fn print_metrics(metrics: Res<ControlMetrics>, scenario: Res<ScenarioConfig>) {
    if metrics.speed_history.is_empty() {
        return;
    }

    let target_speed = metrics
        .speed_history
        .iter()
        .map(|(_, v)| *v)
        .fold(f64::NEG_INFINITY, f64::max);

    let rt = core_metrics::rise_time(&metrics.speed_history, target_speed);
    let st = core_metrics::settling_time(&metrics.speed_history, target_speed, 0.02);
    let os = core_metrics::overshoot_pct(&metrics.speed_history, target_speed);

    let cte =
        core_metrics::cross_track_error(&metrics.actual_positions, &metrics.reference_positions);
    let he = core_metrics::heading_error(&metrics.actual_headings, &metrics.reference_headings);

    let cte_mean = mean(&cte);
    let cte_max = cte.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let he_mean = mean(&he.iter().map(|v| v.abs()).collect::<Vec<_>>());
    let he_max = he.iter().map(|v| v.abs()).fold(f64::NEG_INFINITY, f64::max);

    info!("=== Control Metrics ===");
    info!("  Rise time       : {}", fmt_opt(rt, "s"));
    info!("  Settling time   : {}", fmt_opt(st, "s"));
    info!("  Overshoot       : {:.1} %", os);
    info!(
        "  Cross-track err : mean {:.3} m  max {:.3} m",
        cte_mean, cte_max
    );
    info!(
        "  Heading error   : mean {:.3} rad  max {:.3} rad",
        he_mean, he_max
    );

    if let Some(ref path) = scenario.metrics.output_path {
        write_csv(path, &metrics, &cte, &he);
    }
}

fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.iter().sum::<f64>() / v.len() as f64
}

fn fmt_opt(v: Option<f64>, unit: &str) -> String {
    match v {
        Some(t) => format!("{:.3} {}", t, unit),
        None => "N/A".to_string(),
    }
}

fn write_csv(path: &str, metrics: &ControlMetrics, cte: &[f64], he: &[f64]) {
    use std::fmt::Write as _;
    let mut out = String::from("timestamp_s,speed_mps,cte_m,heading_err_rad\n");
    for (i, &(t, speed)) in metrics.speed_history.iter().enumerate() {
        let c = cte.get(i).copied().unwrap_or(0.0);
        let h = he.get(i).copied().unwrap_or(0.0);
        let _ = writeln!(out, "{t:.4},{speed:.4},{c:.4},{h:.4}");
    }
    if let Err(e) = std::fs::write(path, out) {
        error!("[Metrics] Failed to write CSV to '{}': {}", path, e);
    } else {
        info!("[Metrics] Results written to '{}'", path);
    }
}
