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
use crate::metrics::{final_pos_e_id, final_pos_n_id, RunMetrics};

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
/// final position, stamps it with the run's identity, and stores one
/// `RunMetrics` in the collector before `app.run()` drains the world.
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

    // STOPGAP: a hand-checkable stand-in metric. The ground-truth final
    // position (ENU east/north) is trivially verifiable and directly reflects
    // whether the agent received its goal and drove: an agent that never moved
    // reads near its origin, one that ran reads far from it. `pose` is already
    // ENU, so `.x`/`.y` are east/north with no frame conversion here.
    // Replaced once producers emit their own named scalars to a telemetry sink.
    let t = &ground_truth.pose.translation;

    let mut run_metrics = RunMetrics::new(metadata.run_index, metadata.seed);
    run_metrics.insert(final_pos_e_id(), t.x);
    run_metrics.insert(final_pos_n_id(), t.y);

    collector.store(run_metrics);

    tracing::info!(final_e = t.x, final_n = t.y, "run metrics summarized");
}
