//! The metrics summarizer: one Bevy system that, at the end of a run, reduces
//! the world to a single [`RunMetrics`] and deposits it in the
//! [`MetricsCollector`] so the host can read it after the run.
//!
//! Two parts with different lifespans:
//! - **The plumbing is permanent.** Moving a `RunMetrics` out of the draining
//!   world via the shared collector is the mechanism a future telemetry sink
//!   generalizes (one record → a stream of named scalars).
//! - **The scalar read is a stopgap.** Reaching into `GroundTruthState` and
//!   computing one number is a stand-in producer. It is deleted once producers
//!   emit their own named scalars to a sink; nothing else here changes when it
//!   goes.

use super::{MetricsCollector, RunMetadata};
use crate::metrics::{final_speed_id, RunMetrics};

use helios_sim::prelude::AppState;
use helios_sim::simulation::core::components::GroundTruthState;

use bevy::prelude::*;

/// Registers the summarizer at `OnEnter(AppState::Flushing)`. Added by the bin
/// opt-in, alongside `TestRunnerPlugin`.
pub struct MetricsPlugin;

impl Plugin for MetricsPlugin {
    fn build(&self, app: &mut App) {
        // No explicit ordering against the report finalizer: both run on the
        // same `OnEnter(Flushing)` hook but touch disjoint state (this system:
        // GroundTruthState → collector; the finalizer: buses → report), so the
        // unspecified order between them does not matter.
        app.add_systems(OnEnter(AppState::Flushing), summarize_metrics_system);
    }
}

/// Runs once at `OnEnter(AppState::Flushing)`. Reads the agent's ground-truth
/// velocity, stamps it with the run's identity, and stores one `RunMetrics` in
/// the collector before `app.run()` drains the world.
fn summarize_metrics_system(
    query: Query<&GroundTruthState>,
    metadata: Res<RunMetadata>,
    collector: Res<MetricsCollector>,
) {
    // Single agent today: exactly one body carries `GroundTruthState`, so
    // `single()` is the right shape. Many agents would mean one `RunMetrics`
    // each, changing the collector's shape rather than this read.
    // No unwrap: a missing body skips the summary rather than crashing the
    // flush; the run still finalizes through the separate report system.
    let Ok(ground_truth) = query.single() else {
        tracing::warn!("metrics summarizer: no unique GroundTruthState; skipping");
        return;
    };

    // STOPGAP: a hand-checkable stand-in metric. Final true speed is trivially
    // verifiable and varies with the run seed (so seeded runs genuinely
    // diverge), and commits nothing to a specific control-metrics design.
    // Replaced once producers emit their own named scalars to a telemetry sink.
    let final_speed = ground_truth.linear_velocity.norm();

    let mut run_metrics = RunMetrics::new(metadata.run_index, metadata.seed);
    run_metrics.insert(final_speed_id(), final_speed);

    collector.store(run_metrics);

    tracing::info!(final_speed, "run metrics summarized");
}
