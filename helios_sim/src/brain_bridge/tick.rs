//! Advances every agent's `AutonomyPipeline` one step per `FixedUpdate`.
//!
//! `run_pipeline_tick` runs in `SimulationSet::BrainTick` and executes every
//! DAG node in topological order — the whole brain, not just the estimator.
//! All bus slots are last-known-good; consumers dedupe by `Stamped::timestamp`
//! internally where exactly-once semantics are required.

use crate::brain_bridge::components::{AutonomyPipelineComponent, TfServiceComponent};
use crate::core::run_clock::RunClock;

use bevy::prelude::*;

/// Ticks every agent's `AutonomyPipeline` once per `FixedUpdate`.
pub fn run_pipeline_tick(
    mut query: Query<(&AutonomyPipelineComponent, &mut TfServiceComponent)>,
    time: Res<Time>,
    clock: RunClock,
) {
    // Runs only while `Running`, when the clock has started.
    let Some(now) = clock.now() else {
        return;
    };
    let dt = time.delta_secs_f64();

    for (pipeline_comp, mut service_comp) in &mut query {
        // Fold last tick's dual-published edge samples into the estimated tree,
        // then hand every node a read-only view of it. The `&mut` fold and the
        // shared provider borrow cannot overlap, so all nodes this tick query one
        // frozen buffer regardless of execution order — and, wired only from edge
        // channels, that buffer cannot reach the sim's truth.
        service_comp.0.fold(pipeline_comp.0.bus());
        pipeline_comp.0.tick(now, dt, service_comp.0.as_provider());
    }
}
