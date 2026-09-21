//! Advances every agent's `AutonomyPipeline` one step per `FixedUpdate`.
//!
//! `run_pipeline_tick` runs in `SimulationSet::BrainTick` and executes every
//! DAG node in topological order — the whole brain, not just the estimator.
//! All bus slots are last-known-good; consumers dedupe by `Stamped::timestamp`
//! internally where exactly-once semantics are required.

use bevy::prelude::*;

use helios_core::data::primitives::MonotonicTime;

use crate::brain_bridge::components::{AutonomyPipelineComponent, TfServiceComponent};

/// Ticks every agent's `AutonomyPipeline` once per `FixedUpdate`.
pub fn run_pipeline_tick(
    mut query: Query<(&AutonomyPipelineComponent, &mut TfServiceComponent)>,
    time: Res<Time>,
) {
    let dt = time.delta_secs_f64();
    let elapsed = time.elapsed_secs_f64();

    for (pipeline_comp, mut service_comp) in &mut query {
        // Fold last tick's dual-published edge samples into the estimated tree,
        // then hand every node a read-only view of it. The `&mut` fold and the
        // shared provider borrow cannot overlap, so all nodes this tick query one
        // frozen buffer regardless of execution order — and, wired only from edge
        // channels, that buffer cannot reach the sim's truth tree.
        service_comp.0.fold(pipeline_comp.0.bus());
        pipeline_comp
            .0
            .tick(MonotonicTime(elapsed), dt, service_comp.0.as_provider());
    }
}
